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

// A state slot holds just the state word, rounded up to a whole number of
// flash words: 16 on L4, 32 on H7. (Signature and count used to live here;
// they moved into per-instance layout records when one page pair started
// serving multiple instances.)
constexpr size_t SLOT_SIZE = (FLASH_WORD >= 16) ? FLASH_WORD : 16;
constexpr uint8_t NUM_SLOTS = 3;  // AVAILABLE (implicit) -> IN_USE -> FULL
constexpr size_t HEADER_SIZE = NUM_SLOTS * SLOT_SIZE;

constexpr uint32_t ERASED_WORD   = 0xFFFFFFFFu;
constexpr uint32_t SECTOR_IN_USE = 0x494E5553u; // "INUS"
constexpr uint32_t SECTOR_FULL   = 0x46554C4Cu; // "FULL"

constexpr uint8_t MARKER_ERASED = 0xFFu;
constexpr uint8_t MARKER_VALUE  = 0x53u; // 'S' -- a parameter value
constexpr uint8_t MARKER_LAYOUT = 0x4Cu; // 'L' -- an instance's layout

/*
    Areas. Only AREA_PARAM is implemented; the field exists so bulk storage
    (mission data and similar) can be added later without another on-flash
    format change.

    Before using one, note that a value record carries 4 payload bytes in a
    whole flash word: 50% efficiency on L4, 12.5% on H7. Storing kilobytes of
    mission data through *this* record type would burn the log very quickly.
    A bulk area wants its own record type carrying a longer payload -- on H7
    that is 24 usable bytes per word rather than 4. The marker byte is what
    lets a second record type coexist with these.
*/
constexpr uint8_t AREA_PARAM = 0x0u;

constexpr uint8_t make_tag(uint8_t area, uint8_t instance)
{
    return (uint8_t)((area << 4) | (instance & 0x0Fu));
}
constexpr uint8_t tag_area(uint8_t tag)     { return (uint8_t)(tag >> 4); }
constexpr uint8_t tag_instance(uint8_t tag) { return (uint8_t)(tag & 0x0Fu); }

// ---- On-flash layouts -----------------------------------------------------

/*
    One sector state transition. Written into the next free slot -- never
    rewritten, so the H7 write-once-per-word rule holds.
*/
struct slot_fields
{
    uint32_t state;
};
static_assert(sizeof(slot_fields) <= SLOT_SIZE, "slot_fields must fit a slot");

/*
    One log entry. Eight bytes: exactly a record on L4/G4/F4, and the leading
    eight bytes of the 32-byte H7 flash word (the rest is left erased).

    marker == MARKER_VALUE : a = parameter index, b = float bits
    marker == MARKER_LAYOUT: a = parameter count, b = layout signature
*/
struct record_fields
{
    uint8_t  marker;
    uint8_t  tag;      // area << 4 | instance
    uint16_t a;
    uint32_t b;
};
static_assert(sizeof(record_fields) == 8, "record_fields must be 8 bytes");
static_assert(sizeof(record_fields) <= RECORD_SIZE, "record must fit a flash word");

// ---- Replay image ---------------------------------------------------------

/*
    Reconstructed state of one instance. `written` marks which indices the log
    actually carried, so compaction rewrites only those -- an index that was
    never saved must stay absent, or a later load() would overwrite the
    caller's code default with a zero.

    Static rather than stack: ~270 bytes per instance, and compaction is the
    worst possible moment to overflow the stack.
*/
struct instance_image
{
    bool     present;
    bool     valid;      // current layout record matches what we are replaying
    uint16_t count;
    uint32_t signature;
    uint64_t written;
    float    values[DroneCAN_Storage::MAX_PARAMS];
};

instance_image g_image[DroneCAN_Storage::MAX_INSTANCES];

// ---- Resolved log position (rebuilt by begin()) ---------------------------

struct log_state
{
    bool     valid;
    uint32_t primary;       // first page of the pair, as passed in
    uint32_t active_page;
    uint32_t spare_page;
    uint32_t write_offset;  // byte offset of the next free record slot
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

bool write_sector_state(uint32_t page, uint32_t state)
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
        slot.state = state;
        return flash_write_unit(base + i * SLOT_SIZE, &slot, sizeof(slot), SLOT_SIZE);
    }
    return false; // all slots consumed
}

// ---- Log records ----------------------------------------------------------

bool append(uint32_t page, uint32_t offset, uint8_t marker, uint8_t tag,
            uint16_t a, uint32_t b)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    record_fields rec;
    rec.marker = marker;
    rec.tag    = tag;
    rec.a      = a;
    rec.b      = b;
    return flash_write_unit(base + offset, &rec, sizeof(rec), RECORD_SIZE);
}

bool append_value(uint32_t page, uint32_t offset, uint8_t instance,
                  size_t index, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return append(page, offset, MARKER_VALUE, make_tag(AREA_PARAM, instance),
                  (uint16_t)index, bits);
}

bool append_layout(uint32_t page, uint32_t offset, uint8_t instance,
                   size_t count, uint32_t signature)
{
    return append(page, offset, MARKER_LAYOUT, make_tag(AREA_PARAM, instance),
                  (uint16_t)count, signature);
}

bool room_for(uint32_t page, uint32_t offset, size_t records)
{
    return offset + records * RECORD_SIZE <= stm32_flash_getpagesize(page);
}

/*
    Replay the whole log in `page` into g_image, for every instance present.

    A layout record (re)declares an instance's parameter list. Value records
    apply to whatever layout is current for their instance at that point in
    the log, so a firmware update that changes one instance's parameters
    invalidates only that instance's earlier values -- the other instance's
    records, interleaved in the same log, are unaffected.
*/
bool replay_all(uint32_t page, uint32_t *out_write_offset)
{
    const uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    memset(g_image, 0, sizeof(g_image));

    const uint32_t page_size = stm32_flash_getpagesize(page);
    uint32_t offset = HEADER_SIZE;

    while (offset + RECORD_SIZE <= page_size) {
        record_fields rec;
        flash_read(base + offset, &rec, sizeof(rec));
        if (rec.marker == MARKER_ERASED) {
            break; // append-only: first erased slot ends the log
        }
        offset += RECORD_SIZE;

        if (tag_area(rec.tag) != AREA_PARAM) {
            continue; // a future area; not ours to interpret
        }
        const uint8_t inst = tag_instance(rec.tag);
        if (inst >= DroneCAN_Storage::MAX_INSTANCES) {
            continue;
        }
        instance_image &img = g_image[inst];

        if (rec.marker == MARKER_LAYOUT) {
            // A different layout supersedes everything logged before it.
            if (img.present && (img.count != rec.a || img.signature != rec.b)) {
                img.written = 0;
                memset(img.values, 0, sizeof(img.values));
            }
            img.present   = true;
            img.count     = rec.a;
            img.signature = rec.b;
        } else if (rec.marker == MARKER_VALUE) {
            if (img.present && rec.a < img.count &&
                rec.a < DroneCAN_Storage::MAX_PARAMS) {
                memcpy(&img.values[rec.a], &rec.b, sizeof(float));
                img.written |= (uint64_t)1u << rec.a;
            }
        }
    }

    if (out_write_offset != nullptr) {
        *out_write_offset = offset;
    }
    return true;
}

/*
    Records a full snapshot of g_image needs: one layout record per present
    instance plus one value record per written index.
*/
size_t image_record_count()
{
    size_t n = 0;
    for (size_t i = 0; i < DroneCAN_Storage::MAX_INSTANCES; i++) {
        if (!g_image[i].present) {
            continue;
        }
        n++; // layout record
        for (size_t k = 0; k < DroneCAN_Storage::MAX_PARAMS; k++) {
            if (g_image[i].written & ((uint64_t)1u << k)) {
                n++;
            }
        }
    }
    return n;
}

/*
    Compact g_image into the spare page and make it active.

    This is deliberately instance-agnostic: whichever instance triggers a
    compaction, every instance's live values must survive it. That is why
    g_image is reconstructed by a full replay first, rather than the caller
    handing in its own value array.

    Order matters for power-loss recovery, and follows AP_FlashStorage:
    records first, then mark the old page FULL, then mark the new one IN_USE.
    A crash between the last two leaves (FULL, AVAILABLE), which begin()
    detects and redoes.

    The old page is NOT erased here -- that is the point. It is erased by the
    next begin(), i.e. at boot.
*/
bool switch_sectors()
{
    const uint32_t old_page = g_log.active_page;
    const uint32_t new_page = g_log.spare_page;

    // The spare should already be blank (erased at boot). If a previous
    // compaction in this same power cycle left it dirty, we have no choice
    // but to pay for the erase inline.
    if (!stm32_flash_ispageerased(new_page) && !stm32_flash_erasepage(new_page)) {
        return false;
    }
    if (!room_for(new_page, HEADER_SIZE, image_record_count())) {
        return false; // live set cannot fit in a single page
    }

    uint32_t offset = HEADER_SIZE;
    for (size_t i = 0; i < DroneCAN_Storage::MAX_INSTANCES; i++) {
        const instance_image &img = g_image[i];
        if (!img.present) {
            continue;
        }
        if (!append_layout(new_page, offset, (uint8_t)i, img.count, img.signature)) {
            return false;
        }
        offset += RECORD_SIZE;

        for (size_t k = 0; k < DroneCAN_Storage::MAX_PARAMS; k++) {
            if (!(img.written & ((uint64_t)1u << k))) {
                continue; // never written: must stay absent
            }
            if (!append_value(new_page, offset, (uint8_t)i, k, img.values[k])) {
                return false;
            }
            offset += RECORD_SIZE;
        }
    }

    if (!write_sector_state(old_page, SECTOR_FULL)) {
        return false;
    }
    if (!write_sector_state(new_page, SECTOR_IN_USE)) {
        return false;
    }

    g_log.active_page  = new_page;
    g_log.spare_page   = old_page;
    g_log.write_offset = offset;
    return true;
}

/*
    Resolve which page of the pair is active, replay it into g_image, and
    perform any recovery the previous power cycle left outstanding.
*/
bool begin(uint32_t page)
{
    const uint32_t page_a = page;
    const uint32_t page_b = page + 1;
    if (stm32_flash_getpageaddr(page_a) == 0 || stm32_flash_getpageaddr(page_b) == 0) {
        return false;
    }

    const uint32_t state_a = read_sector_state(page_a);
    const uint32_t state_b = read_sector_state(page_b);

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

    g_log.valid   = true;
    g_log.primary = page;

    if (!have_active) {
        // Nothing usable: fresh part, both FULL, or both IN_USE. Start over.
        if (!stm32_flash_ispageerased(page_a) && !stm32_flash_erasepage(page_a)) {
            return false;
        }
        if (!stm32_flash_ispageerased(page_b) && !stm32_flash_erasepage(page_b)) {
            return false;
        }
        // Claim page_a before any record is appended. Without this the header
        // stays erased and the next begin() would see two AVAILABLE pages and
        // wipe the log we are about to write.
        if (!write_sector_state(page_a, SECTOR_IN_USE)) {
            return false;
        }
        memset(g_image, 0, sizeof(g_image));
        g_log.active_page  = page_a;
        g_log.spare_page   = page_b;
        g_log.write_offset = HEADER_SIZE;
        return true;
    }

    uint32_t write_offset = HEADER_SIZE;
    if (!replay_all(active, &write_offset)) {
        return false;
    }

    g_log.active_page  = active;
    g_log.spare_page   = spare;
    g_log.write_offset = write_offset;

    if (active_is_full) {
        // Interrupted compaction: the spare may hold a partial copy. Erase it
        // and redo the switch from what we just replayed.
        if (!stm32_flash_ispageerased(spare) && !stm32_flash_erasepage(spare)) {
            return false;
        }
        if (image_record_count() > 0) {
            return switch_sectors();
        }
        if (!write_sector_state(spare, SECTOR_IN_USE)) {
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
    save() arriving first re-derives it.
*/
bool ensure_begun(uint32_t page)
{
    if (g_log.valid && g_log.primary == page) {
        return true;
    }
    return begin(page);
}

/*
    Append `n` records via `emit`, compacting first if they will not fit.
    `emit(offset_index)` writes record number i at the current write offset.
*/
bool ensure_room(size_t n)
{
    if (room_for(g_log.active_page, g_log.write_offset, n)) {
        return true;
    }
    // Replay so compaction carries every instance, not just the caller's.
    if (!replay_all(g_log.active_page, nullptr)) {
        return false;
    }
    return switch_sectors();
}

/*
    Make sure the log carries a current layout record for this instance, so
    subsequent value records are interpreted against the right parameter list.
    Appends one only when the declared layout differs from what is already in
    effect, so this costs nothing in the steady state.
*/
bool ensure_layout(uint8_t instance, size_t count, uint32_t signature)
{
    instance_image &img = g_image[instance];
    if (img.present && img.count == (uint16_t)count && img.signature == signature) {
        return true;
    }
    if (!ensure_room(1)) {
        return false;
    }
    if (!append_layout(g_log.active_page, g_log.write_offset, instance,
                       count, signature)) {
        return false;
    }
    g_log.write_offset += RECORD_SIZE;

    if (img.present && (img.count != (uint16_t)count || img.signature != signature)) {
        img.written = 0;
        memset(img.values, 0, sizeof(img.values));
    }
    img.present   = true;
    img.count     = (uint16_t)count;
    img.signature = signature;
    return true;
}

void note_written(uint8_t instance, size_t index, float value)
{
    instance_image &img = g_image[instance];
    if (index < DroneCAN_Storage::MAX_PARAMS) {
        img.values[index] = value;
        img.written |= (uint64_t)1u << index;
    }
}

bool args_ok(size_t count, uint8_t instance)
{
    return count != 0 && count <= DroneCAN_Storage::MAX_PARAMS &&
           instance < DroneCAN_Storage::MAX_INSTANCES;
}

} // namespace

#if defined(STORAGE_HOST_TEST)
/*
    Test-only hook. g_log and g_image cache state for the lifetime of a power
    cycle; on target both are cleared by reset. The host tests need to
    simulate that reboot when they wipe the flash model.
*/
void dronecan_storage_test_reset_cache()
{
    g_log = log_state{};
    memset(g_image, 0, sizeof(g_image));
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
    One pair for the whole board: (base, base+1). Instances share it and are
    told apart by the tag byte in each record, so a dual-CAN node needs no
    extra flash -- which on H7 would cost another 256 KB.
*/
uint32_t DroneCAN_Storage::default_page()
{
    return STORAGE_FLASH_PAGE;
}

bool DroneCAN_Storage::load(float *values, size_t count, uint32_t page,
                            uint8_t instance, uint32_t signature)
{
    if (values == nullptr || !args_ok(count, instance)) {
        return false;
    }
    if (!ensure_begun(page)) {
        failed_ = true;
        return false;
    }
    failed_ = false;

    const instance_image &img = g_image[instance];
    if (!img.present || img.count != (uint16_t)count || img.signature != signature) {
        return false; // nothing stored for this instance, or a different layout
    }
    if (img.written == 0) {
        return false;
    }
    // Only indices the log actually carried; the rest keep the caller's
    // code defaults.
    for (size_t i = 0; i < count; i++) {
        if (img.written & ((uint64_t)1u << i)) {
            values[i] = img.values[i];
        }
    }
    return true;
}

bool DroneCAN_Storage::save(size_t index, float value, size_t total_count,
                            uint32_t page, uint8_t instance, uint32_t signature)
{
    if (index >= total_count || !args_ok(total_count, instance)) {
        return false;
    }
    if (!ensure_begun(page)) {
        failed_ = true;
        return false;
    }
    if (!ensure_layout(instance, total_count, signature) || !ensure_room(1)) {
        failed_ = true;
        return false;
    }
    if (!append_value(g_log.active_page, g_log.write_offset, instance, index, value)) {
        failed_ = true;
        return false;
    }
    g_log.write_offset += RECORD_SIZE;
    note_written(instance, index, value);
    return true;
}

bool DroneCAN_Storage::save_all(const float *values, size_t count, uint32_t page,
                                uint8_t instance, uint32_t signature)
{
    if (values == nullptr || !args_ok(count, instance)) {
        return false;
    }
    if (!ensure_begun(page)) {
        failed_ = true;
        return false;
    }
    if (!ensure_layout(instance, count, signature) || !ensure_room(count)) {
        failed_ = true;
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (!append_value(g_log.active_page, g_log.write_offset, instance, i, values[i])) {
            failed_ = true;
            return false;
        }
        g_log.write_offset += RECORD_SIZE;
        note_written(instance, i, values[i]);
    }
    return true;
}
