#include "storage.h"
#include "flash.h"
#include <Arduino.h>
#include <string.h>

// Mirror flash.cpp's family normalisation. Board JSONs define STM32H7xx for
// both H7 boards and STM32H7 for MicroNodePlus only, so accept either.
#if defined(STM32H7xx) && !defined(STM32H7)
#define STM32H7
#endif
#if defined(STM32L4xx) && !defined(STM32L4)
#define STM32L4
#endif

#ifndef STORAGE_FLASH_PAGE
#error "STORAGE_FLASH_PAGE must be defined by the board (see br_platformio_hwdef boards/*.json)"
#endif

bool DroneCAN_Storage::failed_ = false;

namespace
{

// ---- Family geometry ------------------------------------------------------

#if defined(STM32H7)
// H7 programs a 256-bit flash word, once per erase (ECC spans the word).
constexpr size_t FLASH_WORD = 32;
#else
// L4/G4 program a 64-bit double word. F4 can do less, but 8 is a safe
// common denominator and keeps one record layout everywhere.
constexpr size_t FLASH_WORD = 8;
#endif

// A record is exactly one flash word: the smallest independently
// programmable unit, so appending never disturbs an earlier record.
constexpr size_t RECORD_SIZE = FLASH_WORD;

// A state slot holds state + signature + count (12 bytes), rounded up to a
// whole number of flash words: 16 on L4, 32 on H7.
constexpr size_t SLOT_SIZE = (FLASH_WORD >= 16) ? FLASH_WORD : 16;
constexpr uint8_t NUM_SLOTS = 3;  // AVAILABLE (implicit) -> IN_USE -> FULL
constexpr size_t HEADER_SIZE = NUM_SLOTS * SLOT_SIZE;

constexpr uint32_t ERASED_WORD  = 0xFFFFFFFFu;
constexpr uint32_t SECTOR_IN_USE = 0x494E5553u; // "INUS"
constexpr uint32_t SECTOR_FULL   = 0x46554C4Cu; // "FULL"
constexpr uint32_t SECTOR_FOREIGN = 0x00000001u; // synthesised, never on flash
constexpr uint16_t RECORD_MARKER = 0x5243u;      // "RC"

// ---- On-flash layouts -----------------------------------------------------

/*
    One sector state transition. Written into the next free slot -- never
    rewritten, so the H7 write-once-per-word rule holds.
*/
struct slot_fields
{
    uint32_t state;
    uint32_t signature;
    uint32_t count;
};
static_assert(sizeof(slot_fields) <= SLOT_SIZE, "slot_fields must fit a slot");

/*
    One logged parameter write. Eight bytes: exactly a record on L4/G4/F4,
    and the leading eight bytes of the 32-byte H7 flash word (the remainder
    is left erased).
*/
struct record_fields
{
    uint16_t marker;
    uint16_t index;
    float    value;
};
static_assert(sizeof(record_fields) == 8, "record_fields must be 8 bytes");
static_assert(sizeof(record_fields) <= RECORD_SIZE, "record must fit a flash word");

// ---- Resolved log position (rebuilt by begin()) ---------------------------

struct log_state
{
    bool     valid;
    uint32_t primary;       // first page of the pair, as passed in
    uint32_t active_page;
    uint32_t spare_page;
    uint32_t write_offset;  // byte offset of the next free record slot
    uint32_t signature;
};

log_state g_log = {};

// ---- Flash access ---------------------------------------------------------

/*
    Read memory-mapped flash, defeating any D-cache. Flash changes underneath
    the cache when we program, so every read of a region we may have written
    goes through here.
*/
void flash_read(uint32_t addr, void *dst, size_t len)
{
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)(addr & ~0x1Fu),
                                 (int32_t)(len + 32));
#endif
    memcpy(dst, (const void *)(uintptr_t)addr, len);
}

/*
    Program one unit (record or slot). `fields` is copied into a unit-sized
    buffer whose remaining bytes stay 0xFF, so padding is left erased.
*/
bool flash_write_unit(uint32_t addr, const void *fields, size_t len, size_t unit)
{
    uint8_t buf[SLOT_SIZE > RECORD_SIZE ? SLOT_SIZE : RECORD_SIZE];
    memset(buf, 0xFF, unit);
    memcpy(buf, fields, len);
    return stm32_flash_write(addr, buf, (uint32_t)unit);
}

// ---- Sector state ---------------------------------------------------------

/*
    Current state of a page: the last slot that has been programmed. An
    untouched (erased) page reads back as all-ones -> AVAILABLE.
*/
uint32_t read_sector_state(uint32_t page)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return ERASED_WORD;
    }
    uint32_t state = ERASED_WORD;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        slot_fields slot;
        flash_read(base + i * SLOT_SIZE, &slot, sizeof(slot));
        if (slot.state == ERASED_WORD) {
            break; // slots fill in order; first erased one ends the chain
        }
        state = slot.state;
    }
    return state;
}

bool write_sector_state(uint32_t page, uint32_t state, uint32_t signature, uint32_t count)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        slot_fields slot;
        flash_read(base + i * SLOT_SIZE, &slot, sizeof(slot));
        if (slot.state != ERASED_WORD) {
            continue;
        }
        slot.state     = state;
        slot.signature = signature;
        slot.count     = count;
        return flash_write_unit(base + i * SLOT_SIZE, &slot, sizeof(slot), SLOT_SIZE);
    }
    return false; // all slots consumed
}

// ---- Log records ----------------------------------------------------------

bool append_record(uint32_t page, uint32_t offset, size_t index, float value)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    record_fields rec;
    rec.marker = RECORD_MARKER;
    rec.index  = (uint16_t)index;
    rec.value  = value;
    return flash_write_unit(base + offset, &rec, sizeof(rec), RECORD_SIZE);
}

bool room_for(uint32_t page, uint32_t offset, size_t records)
{
    return offset + records * RECORD_SIZE <= stm32_flash_getpagesize(page);
}

/*
    Walk the log in `page`, applying every valid record to `values`.
    `values` may be null to scan for the write offset only.
*/
bool replay_log(uint32_t page, float *values, size_t count,
                uint32_t *out_write_offset, bool *out_found_any)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    const uint32_t page_size = stm32_flash_getpagesize(page);
    uint32_t offset = HEADER_SIZE;
    bool found_any = false;

    while (offset + RECORD_SIZE <= page_size) {
        record_fields rec;
        flash_read(base + offset, &rec, sizeof(rec));
        if (rec.marker == 0xFFFFu) {
            break; // append-only: first erased slot ends the log
        }
        if (rec.marker == RECORD_MARKER && rec.index < count) {
            if (values != nullptr) {
                values[rec.index] = rec.value;
            }
            found_any = true;
        }
        offset += RECORD_SIZE;
    }

    if (out_write_offset != nullptr) {
        *out_write_offset = offset;
    }
    if (out_found_any != nullptr) {
        *out_found_any = found_any;
    }
    return true;
}

/*
    Compact `values` into the spare page and make it active.

    Order matters for power-loss recovery, and follows AP_FlashStorage:
    records first, then mark the old page FULL, then mark the new one
    IN_USE. A crash between the last two leaves (FULL, AVAILABLE), which
    begin() detects and redoes.

    The old page is NOT erased here -- that is the point. It is erased by
    the next begin(), i.e. at boot.
*/
bool switch_sectors(const float *values, size_t count, uint32_t signature)
{
    const uint32_t old_page = g_log.active_page;
    const uint32_t new_page = g_log.spare_page;

    // The spare should already be blank (erased at boot). If a previous
    // compaction in this same power cycle left it dirty, we have no choice
    // but to pay for the erase inline.
    if (!stm32_flash_ispageerased(new_page) && !stm32_flash_erasepage(new_page)) {
        return false;
    }
    if (!room_for(new_page, HEADER_SIZE, count)) {
        return false; // parameter set cannot fit in a single page
    }

    uint32_t offset = HEADER_SIZE;
    for (size_t i = 0; i < count; i++) {
        if (!append_record(new_page, offset, i, values[i])) {
            return false;
        }
        offset += RECORD_SIZE;
    }

    if (!write_sector_state(old_page, SECTOR_FULL, signature, (uint32_t)count)) {
        return false;
    }
    if (!write_sector_state(new_page, SECTOR_IN_USE, signature, (uint32_t)count)) {
        return false;
    }

    g_log.active_page  = new_page;
    g_log.spare_page   = old_page;
    g_log.write_offset = offset;
    return true;
}

/*
    Does this page's header match the running firmware's parameter layout?
    A page written by another layout holds values whose indices mean
    something else, so it must not be replayed.
*/
bool layout_matches(uint32_t page, uint32_t signature, size_t count)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    slot_fields slot;
    flash_read(base, &slot, sizeof(slot));
    return slot.signature == signature && slot.count == (uint32_t)count;
}

/*
    Resolve which page of the pair is active, replay it into `values`, and
    perform any recovery the previous power cycle left outstanding.
*/
bool begin(uint32_t page, float *values, size_t count, uint32_t signature,
           bool *out_loaded)
{
    *out_loaded = false;

    const uint32_t page_a = page;
    const uint32_t page_b = page + 1;
    if (stm32_flash_getpageaddr(page_a) == 0 || stm32_flash_getpageaddr(page_b) == 0) {
        return false;
    }

    uint32_t state_a = read_sector_state(page_a);
    uint32_t state_b = read_sector_state(page_b);
    if (state_a != ERASED_WORD && !layout_matches(page_a, signature, count)) {
        state_a = SECTOR_FOREIGN;
    }
    if (state_b != ERASED_WORD && !layout_matches(page_b, signature, count)) {
        state_b = SECTOR_FOREIGN;
    }

    uint32_t active = 0, spare = 0;
    bool have_active = false;
    bool active_is_full = false;

    if (state_a == SECTOR_IN_USE && state_b != SECTOR_IN_USE) {
        active = page_a; spare = page_b; have_active = true;
    } else if (state_b == SECTOR_IN_USE && state_a != SECTOR_IN_USE) {
        active = page_b; spare = page_a; have_active = true;
    } else if (state_a == SECTOR_FULL && state_b == ERASED_WORD) {
        // Crashed after marking A full, before B was marked in use.
        active = page_a; spare = page_b; have_active = true; active_is_full = true;
    } else if (state_b == SECTOR_FULL && state_a == ERASED_WORD) {
        active = page_b; spare = page_a; have_active = true; active_is_full = true;
    }

    g_log.valid     = true;
    g_log.primary   = page;
    g_log.signature = signature;

    if (!have_active) {
        // Nothing usable: fresh part, both FULL, both IN_USE, or a foreign
        // parameter layout. Start over.
        if (!stm32_flash_ispageerased(page_a) && !stm32_flash_erasepage(page_a)) {
            return false;
        }
        if (!stm32_flash_ispageerased(page_b) && !stm32_flash_erasepage(page_b)) {
            return false;
        }
        // Claim page_a before any record is appended. Without this the
        // header stays erased and the next begin() would see two AVAILABLE
        // pages and wipe the log we are about to write.
        if (!write_sector_state(page_a, SECTOR_IN_USE, signature, (uint32_t)count)) {
            return false;
        }
        g_log.active_page  = page_a;
        g_log.spare_page   = page_b;
        g_log.write_offset = HEADER_SIZE;
        return true;
    }

    bool found_any = false;
    uint32_t write_offset = HEADER_SIZE;
    if (!replay_log(active, values, count, &write_offset, &found_any)) {
        return false;
    }

    g_log.active_page  = active;
    g_log.spare_page   = spare;
    g_log.write_offset = write_offset;
    *out_loaded = found_any;

    if (active_is_full) {
        // Interrupted compaction: the spare may hold a partial copy. Erase
        // it and redo the switch from what we just replayed.
        if (!stm32_flash_ispageerased(spare) && !stm32_flash_erasepage(spare)) {
            return false;
        }
        if (found_any) {
            return switch_sectors(values, count, signature);
        }
        if (!write_sector_state(spare, SECTOR_IN_USE, signature, (uint32_t)count)) {
            return false;
        }
        g_log.active_page  = spare;
        g_log.spare_page   = active;
        g_log.write_offset = HEADER_SIZE;
        return true;
    }

    // Steady state: make sure the spare is blank and ready, paying any
    // deferred erase here at boot rather than on a later write.
    if (!stm32_flash_ispageerased(spare) && !stm32_flash_erasepage(spare)) {
        return false;
    }
    return true;
}

/*
    Make sure g_log describes `page`. load() normally does this at boot; a
    save() arriving first re-derives it without disturbing caller state.
*/
bool ensure_begun(uint32_t page, size_t count, uint32_t signature)
{
    if (g_log.valid && g_log.primary == page && g_log.signature == signature) {
        return true;
    }
    bool loaded = false;
    float scratch[DroneCAN_Storage::MAX_PARAMS];
    memset(scratch, 0, sizeof(scratch));
    return begin(page, scratch, count, signature, &loaded);
}

} // namespace

#if defined(STORAGE_HOST_TEST)
/*
    Test-only hook. g_log caches the resolved log position for the lifetime
    of a power cycle; on target it is cleared by reset. The host tests need
    to simulate that reboot when they wipe the flash model.
*/
void dronecan_storage_test_reset_cache()
{
    g_log = log_state{};
}
#endif

// ---- Public API -----------------------------------------------------------

bool DroneCAN_Storage::healthy()
{
    return !failed_;
}

/*
    FNV-1a over the parameter name list. Cheap, no allocation, and good
    enough to detect a reordered/renamed/resized parameter set -- this is a
    layout check, not a security check.
*/
uint32_t DroneCAN_Storage::signature_begin()
{
    return 0x811C9DC5u; // FNV offset basis
}

uint32_t DroneCAN_Storage::signature_feed(uint32_t sig, const char *name)
{
    if (name == nullptr) {
        name = "";
    }
    for (const char *p = name; *p; p++) {
        sig ^= (uint8_t)*p;
        sig *= 0x01000193u;
    }
    // Separator, so {"AB","C"} and {"A","BC"} do not collide
    sig ^= 0xFFu;
    sig *= 0x01000193u;
    return sig;
}

/*
    Storage occupies a contiguous region starting at STORAGE_FLASH_PAGE.
    Each port owns a ping-pong pair: PORT1 -> (base, base+1),
    PORT2 -> (base+2, base+3).
*/
uint32_t DroneCAN_Storage::default_page(uint8_t port_index)
{
    const uint32_t base = STORAGE_FLASH_PAGE;
#if defined(STM32L4)
    // L431 has a single CAN port; every port index maps to the one pair.
    (void)port_index;
    return base;
#else
    return (port_index <= 1) ? (base + port_index * 2) : base;
#endif
}

bool DroneCAN_Storage::load(float *values, size_t count, uint32_t page, uint32_t signature)
{
    if (values == nullptr || count == 0 || count > MAX_PARAMS) {
        return false;
    }
    bool loaded = false;
    if (!begin(page, values, count, signature, &loaded)) {
        failed_ = true;
        return false;
    }
    failed_ = false;
    return loaded;
}

bool DroneCAN_Storage::save(size_t index, float value, size_t total_count,
                            uint32_t page, uint32_t signature)
{
    if (index >= total_count || total_count == 0 || total_count > MAX_PARAMS) {
        return false;
    }
    if (!ensure_begun(page, total_count, signature)) {
        failed_ = true;
        return false;
    }

    if (room_for(g_log.active_page, g_log.write_offset, 1)) {
        const bool ok = append_record(g_log.active_page, g_log.write_offset,
                                      index, value);
        if (ok) {
            g_log.write_offset += RECORD_SIZE;
        } else {
            failed_ = true;
        }
        return ok;
    }

    // Log is full: rebuild the current value set and compact into the spare.
    float values[MAX_PARAMS];
    memset(values, 0, sizeof(values));
    bool found_any = false;
    if (!replay_log(g_log.active_page, values, total_count, nullptr, &found_any)) {
        failed_ = true;
        return false;
    }
    values[index] = value;

    const bool ok = switch_sectors(values, total_count, signature);
    failed_ = !ok;
    return ok;
}

bool DroneCAN_Storage::save_all(const float *values, size_t count,
                                uint32_t page, uint32_t signature)
{
    if (values == nullptr || count == 0 || count > MAX_PARAMS) {
        return false;
    }
    if (!ensure_begun(page, count, signature)) {
        failed_ = true;
        return false;
    }

    bool ok;
    if (room_for(g_log.active_page, g_log.write_offset, count)) {
        ok = true;
        for (size_t i = 0; i < count && ok; i++) {
            ok = append_record(g_log.active_page, g_log.write_offset, i, values[i]);
            if (ok) {
                g_log.write_offset += RECORD_SIZE;
            }
        }
    } else {
        ok = switch_sectors(values, count, signature);
    }
    failed_ = !ok;
    return ok;
}
