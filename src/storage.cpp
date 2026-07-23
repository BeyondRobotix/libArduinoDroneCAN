#include "storage.h"
#include "flash.h"
#include <Arduino.h>
#include <string.h>

// Mirror flash.cpp's family normalisation. Board JSONs define STM32H7xx for
// both H7 boards and STM32H7 for MicroNodePlus only, so accept either.
#if defined(STM32H7xx) && !defined(STM32H7)
#define STM32H7
#endif

bool DroneCAN_Storage::failed_ = false;

bool DroneCAN_Storage::healthy()
{
    return !failed_;
}

/*
    Round n up to the next WRITE_ALIGN boundary.
*/
size_t DroneCAN_Storage::align_up(size_t n)
{
    return (n + WRITE_ALIGN - 1) & ~(WRITE_ALIGN - 1);
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
    Return the flash page for a given port index (0=PORT1, 1=PORT2).
    STORAGE_FLASH_PAGE overrides everything for single-instance builds.
*/
uint32_t DroneCAN_Storage::default_page(uint8_t port_index)
{
#ifdef STORAGE_FLASH_PAGE
    (void)port_index;
    return STORAGE_FLASH_PAGE;
#elif defined(STM32H7)
    // Each port owns a ping-pong pair: PORT1 -> (last, last-1),
    // PORT2 -> (last-2, last-3). Returned value is the first of the pair.
    uint32_t last = stm32_flash_getnumpages() - 1;
    return (port_index <= 1) ? (last - port_index * 2) : last;
#else
    uint32_t last = stm32_flash_getnumpages() - 1;
    return (port_index <= 1) ? (last - port_index) : last;
#endif
}

#if defined(STM32H7)

// ===========================================================================
// H7: log-structured, two-sector ping-pong
// ===========================================================================

namespace
{

constexpr uint32_t ERASED_WORD = 0xFFFFFFFFu;

// Sector states. A state transition writes the next free 32-byte slot --
// an H7 flash word cannot be reprogrammed to clear more bits.
constexpr uint32_t SECTOR_IN_USE = 0x494E5553u; // "INUS"
constexpr uint32_t SECTOR_FULL   = 0x46554C4Cu; // "FULL"

// Marker in the first word of a populated log record.
constexpr uint32_t RECORD_MARKER = 0x5245434Bu; // "RECK"

constexpr size_t FLASH_WORD = 32; // H7 256-bit programming granularity
constexpr uint8_t NUM_SLOTS = 3;  // AVAILABLE -> IN_USE -> FULL

// One sector state transition. Padded to a whole flash word so each
// transition is a single, independent, write-once program operation.
struct sector_slot
{
    uint32_t state;
    uint32_t signature;
    uint32_t pad[6];
};
static_assert(sizeof(sector_slot) == FLASH_WORD, "sector_slot must be one flash word");

// One logged parameter write.
struct record
{
    uint32_t marker;
    uint32_t signature;
    uint16_t index;
    uint16_t count;
    float    value;
    uint32_t pad[4];
};
static_assert(sizeof(record) == FLASH_WORD, "record must be one flash word");

constexpr size_t HEADER_SIZE = NUM_SLOTS * sizeof(sector_slot);

// Resolved log position, rebuilt by load().
struct log_state
{
    bool     valid;         // begin() has run for `primary`
    uint32_t primary;       // page passed in by the caller
    uint32_t active_page;   // sector currently IN_USE
    uint32_t spare_page;    // the other one
    uint32_t write_offset;  // byte offset of the next free record slot
    uint32_t signature;     // signature the active sector was opened with
};

log_state g_log = {};

// Read from memory-mapped flash, defeating the D-cache. Flash contents
// change underneath the cache when we program, so every read of a region
// we may have written goes through here.
void flash_read(uint32_t addr, void *dst, size_t len)
{
    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)(addr & ~0x1Fu), (int32_t)(len + 32));
    memcpy(dst, (const void *)(uintptr_t)addr, len);
}

bool sector_is_erased(uint32_t page)
{
    return stm32_flash_ispageerased(page);
}

/*
    Current state of a sector: the last slot that has been programmed.
    An untouched (erased) sector reads back as all-ones -> AVAILABLE.
*/
uint32_t read_sector_state(uint32_t page, uint32_t *out_signature)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    uint32_t state = ERASED_WORD;
    if (out_signature != nullptr) {
        *out_signature = 0;
    }
    if (base == 0) {
        return ERASED_WORD;
    }
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        sector_slot slot;
        flash_read(base + i * sizeof(sector_slot), &slot, sizeof(slot));
        if (slot.state == ERASED_WORD) {
            break; // slots are filled in order; first erased one ends the chain
        }
        state = slot.state;
        if (out_signature != nullptr) {
            *out_signature = slot.signature;
        }
    }
    return state;
}

/*
    Program the next free header slot with `state`. Returns false if the
    sector has already used all three (should not happen: AVAILABLE is
    implicit, so only IN_USE and FULL ever consume slots).
*/
bool write_sector_state(uint32_t page, uint32_t state, uint32_t signature)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        sector_slot slot;
        flash_read(base + i * sizeof(sector_slot), &slot, sizeof(slot));
        if (slot.state != ERASED_WORD) {
            continue;
        }
        memset(&slot, 0xFF, sizeof(slot)); // leave pad bytes erased
        slot.state = state;
        slot.signature = signature;
        return stm32_flash_write(base + i * sizeof(sector_slot), &slot, sizeof(slot));
    }
    return false;
}

/*
    Walk the log in `page`, applying every valid record to `values`.
    Sets *out_write_offset to the first free record slot.

    `values` may be null to scan for the write offset only. Records whose
    signature does not match are skipped -- they belong to a previous
    firmware's parameter layout.
*/
bool replay_log(uint32_t page, float *values, size_t count, uint32_t signature,
                uint32_t *out_write_offset, bool *out_found_any)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    const uint32_t sector_size = stm32_flash_getpagesize(page);
    uint32_t offset = HEADER_SIZE;
    bool found_any = false;

    while (offset + sizeof(record) <= sector_size) {
        record rec;
        flash_read(base + offset, &rec, sizeof(rec));
        if (rec.marker == ERASED_WORD) {
            break; // append-only: first erased slot is the end of the log
        }
        if (rec.marker == RECORD_MARKER &&
            rec.signature == signature &&
            rec.count == (uint16_t)count &&
            rec.index < count) {
            if (values != nullptr) {
                values[rec.index] = rec.value;
            }
            found_any = true;
        }
        offset += sizeof(record);
    }

    if (out_write_offset != nullptr) {
        *out_write_offset = offset;
    }
    if (out_found_any != nullptr) {
        *out_found_any = found_any;
    }
    return true;
}

bool append_record(uint32_t page, uint32_t offset, size_t index, size_t count,
                   float value, uint32_t signature)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }
    record rec;
    memset(&rec, 0xFF, sizeof(rec)); // pad stays erased
    rec.marker    = RECORD_MARKER;
    rec.signature = signature;
    rec.index     = (uint16_t)index;
    rec.count     = (uint16_t)count;
    rec.value     = value;
    return stm32_flash_write(base + offset, &rec, sizeof(rec));
}

bool room_for(uint32_t page, uint32_t offset, size_t records)
{
    return offset + records * sizeof(record) <= stm32_flash_getpagesize(page);
}

/*
    Compact `values` into the spare sector and make it active.

    Order matters for power-loss recovery, and follows AP_FlashStorage:
    records first, then mark the old sector FULL, then mark the new one
    IN_USE. A crash between the last two steps leaves (FULL, AVAILABLE),
    which begin() detects and redoes.

    The old sector is NOT erased here -- that is the whole point. It is
    erased by the next begin(), i.e. at boot.
*/
bool switch_sectors(const float *values, size_t count, uint32_t signature)
{
    const uint32_t old_page = g_log.active_page;
    const uint32_t new_page = g_log.spare_page;

    // The spare must be blank. Normally it was erased at boot; if a previous
    // compaction in this same power cycle left it dirty we have no choice but
    // to pay for the erase inline.
    if (!sector_is_erased(new_page)) {
        if (!stm32_flash_erasepage(new_page)) {
            return false;
        }
    }

    if (!room_for(new_page, HEADER_SIZE, count)) {
        return false; // parameter set cannot fit in a single sector
    }

    uint32_t offset = HEADER_SIZE;
    for (size_t i = 0; i < count; i++) {
        if (!append_record(new_page, offset, i, count, values[i], signature)) {
            return false;
        }
        offset += sizeof(record);
    }

    if (!write_sector_state(old_page, SECTOR_FULL, signature)) {
        return false;
    }
    if (!write_sector_state(new_page, SECTOR_IN_USE, signature)) {
        return false;
    }

    g_log.active_page  = new_page;
    g_log.spare_page   = old_page;
    g_log.write_offset = offset;
    return true;
}

/*
    Resolve which sector is active, replay it into `values`, and perform any
    recovery the previous power cycle left outstanding (erasing a FULL
    sector, or redoing an interrupted switch).

    Returns true if `values` was populated from a matching-signature log.
*/
bool begin(uint32_t page, float *values, size_t count, uint32_t signature,
           bool *out_loaded)
{
    *out_loaded = false;

    const uint32_t page_a = page;
    const uint32_t page_b = page - 1;

    if (stm32_flash_getpageaddr(page_a) == 0 || page == 0 ||
        stm32_flash_getpageaddr(page_b) == 0) {
        return false;
    }

    const uint32_t state_a = read_sector_state(page_a, nullptr);
    const uint32_t state_b = read_sector_state(page_b, nullptr);

    uint32_t active = 0;
    uint32_t spare  = 0;
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

    g_log.valid       = true;
    g_log.primary     = page;
    g_log.signature   = signature;

    if (!have_active) {
        // Nothing usable (fresh part, both FULL, or both IN_USE). Start over.
        if (!sector_is_erased(page_a) && !stm32_flash_erasepage(page_a)) {
            return false;
        }
        if (!sector_is_erased(page_b) && !stm32_flash_erasepage(page_b)) {
            return false;
        }
        // Claim page_a before any record is appended. Without this the
        // sector header stays erased, and the next begin() would see two
        // AVAILABLE sectors and wipe the log we are about to write.
        if (!write_sector_state(page_a, SECTOR_IN_USE, signature)) {
            return false;
        }
        g_log.active_page  = page_a;
        g_log.spare_page   = page_b;
        g_log.write_offset = HEADER_SIZE;
        return true;
    }

    bool found_any = false;
    uint32_t write_offset = HEADER_SIZE;
    if (!replay_log(active, values, count, signature, &write_offset, &found_any)) {
        return false;
    }

    g_log.active_page  = active;
    g_log.spare_page   = spare;
    g_log.write_offset = write_offset;
    *out_loaded = found_any;

    if (active_is_full) {
        // Interrupted compaction: the spare may hold a partial copy. Erase
        // it and redo the switch from what we just replayed. Only reachable
        // if power was lost inside switch_sectors(), so the erase cost here
        // is acceptable.
        if (!sector_is_erased(spare) && !stm32_flash_erasepage(spare)) {
            return false;
        }
        if (found_any) {
            if (!switch_sectors(values, count, signature)) {
                return false;
            }
        } else {
            // Nothing worth keeping; adopt the blank spare directly.
            if (!write_sector_state(spare, SECTOR_IN_USE, signature)) {
                return false;
            }
            g_log.active_page  = spare;
            g_log.spare_page   = active;
            g_log.write_offset = HEADER_SIZE;
        }
        return true;
    }

    // Steady state: make sure the spare is blank and ready, paying any
    // deferred erase here at boot rather than on a later write.
    if (!sector_is_erased(spare)) {
        if (!stm32_flash_erasepage(spare)) {
            return false;
        }
    }
    return true;
}

/*
    Make sure g_log describes `page`. load() normally does this at boot; a
    save() that arrives first (or after a page change) re-derives it without
    touching the caller's values.
*/
bool ensure_begun(uint32_t page, size_t count, uint32_t signature)
{
    if (g_log.valid && g_log.primary == page && g_log.signature == signature) {
        return true;
    }
    // Scratch buffer: we only want the resolved state, not the values.
    bool loaded = false;
    float scratch[DroneCAN_Storage::MAX_PARAMS];
    memset(scratch, 0, sizeof(scratch));
    return begin(page, scratch, count, signature, &loaded);
}

/*
    Append one record per value. Caller has already checked there is room.
*/
bool append_values(const float *values, size_t count, uint32_t signature)
{
    for (size_t i = 0; i < count; i++) {
        if (!append_record(g_log.active_page, g_log.write_offset, i,
                           count, values[i], signature)) {
            return false;
        }
        g_log.write_offset += sizeof(record);
    }
    return true;
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

bool DroneCAN_Storage::load(float *values, size_t count, uint32_t page, uint32_t signature)
{
    if (values == nullptr || count == 0) {
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
                                      index, total_count, value, signature);
        if (ok) {
            g_log.write_offset += sizeof(record);
        } else {
            failed_ = true;
        }
        return ok;
    }

    // Log is full: rebuild the current value set and compact into the spare.
    float values[MAX_PARAMS];
    memset(values, 0, sizeof(values));
    bool found_any = false;
    if (!replay_log(g_log.active_page, values, total_count, signature, nullptr, &found_any)) {
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
    if (values == nullptr || count == 0) {
        return false;
    }
    if (!ensure_begun(page, count, signature)) {
        failed_ = true;
        return false;
    }

    bool ok;
    if (room_for(g_log.active_page, g_log.write_offset, count)) {
        ok = append_values(values, count, signature);
    } else {
        ok = switch_sectors(values, count, signature);
    }
    failed_ = !ok;
    return ok;
}

#else // !STM32H7

// ===========================================================================
// L4 / G4 / F4: erase the page and rewrite the whole blob (unchanged scheme,
// plus a signature word)
// ===========================================================================

/*
    Load parameter values by reading directly from the memory-mapped flash.
    Returns false (and leaves `values` untouched) when the magic marker or
    the signature does not match -- this preserves code-defined defaults on
    first boot, after a full chip erase, or after a firmware update that
    changed the parameter layout.
*/
bool DroneCAN_Storage::load(float *values, size_t count, uint32_t page, uint32_t signature)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0 || values == nullptr || count == 0) {
        return false;
    }

    // On parts with a D-cache, stale lines can shadow memory-mapped flash.
    size_t region = align_up(count * sizeof(float) + 2 * sizeof(uint32_t));
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    SCB_InvalidateDCache_by_Addr((void *)base, (int32_t)region);
#else
    (void)region;
#endif

    // The magic word sits right after the parameter floats, signature after that
    const size_t magic_offset = count * sizeof(float);
    uint32_t magic = 0;
    uint32_t stored_signature = 0;
    memcpy(&magic, (const void *)(base + magic_offset), sizeof(magic));
    memcpy(&stored_signature, (const void *)(base + magic_offset + sizeof(magic)),
           sizeof(stored_signature));
    if (magic != STORAGE_MAGIC || stored_signature != signature) {
        return false;
    }

    // Read the parameter values straight from flash
    memcpy(values, (const void *)base, count * sizeof(float));
    failed_ = false;
    return true;
}

/*
    Persist a single parameter. Because flash pages must be erased as a
    whole before any byte can be rewritten, we:
      1. read the current blob from flash into RAM
      2. patch the one value that changed
      3. erase + rewrite the full page
*/
bool DroneCAN_Storage::save(size_t index, float value, size_t total_count,
                            uint32_t page, uint32_t signature)
{
    if (index >= total_count || total_count == 0 || total_count > MAX_PARAMS) {
        return false;
    }

    // Build a working copy of all values, starting from flash if it is valid
    float buf[MAX_PARAMS];
    if (!load(buf, total_count, page, signature)) {
        memset(buf, 0, sizeof(buf));
    }

    buf[index] = value;
    return save_all(buf, total_count, page, signature);
}

/*
    Persist every parameter value.
*/
bool DroneCAN_Storage::save_all(const float *values, size_t count,
                                uint32_t page, uint32_t signature)
{
    return write_page(values, count, page, signature);
}

/*
    Erase the storage page and write the parameter blob + magic + signature.
    The buffer written to flash is zero-padded to WRITE_ALIGN so the write
    granularity of every supported family is satisfied.
*/
bool DroneCAN_Storage::write_page(const float *values, size_t count,
                                  uint32_t page, uint32_t signature)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0 || values == nullptr || count == 0 || count > MAX_PARAMS) {
        failed_ = true;
        return false;
    }

    // Total payload: N floats + magic + signature, rounded up
    const size_t raw_size = count * sizeof(float) + 2 * sizeof(uint32_t);
    const size_t buf_size = align_up(raw_size);
    if (buf_size > stm32_flash_getpagesize(page)) {
        failed_ = true;
        return false;
    }

    // Build an aligned write buffer (zero-padded)
    uint8_t buf[align_up_const(MAX_PARAMS * sizeof(float) + 2 * sizeof(uint32_t))];
    memset(buf, 0, buf_size);
    memcpy(buf, values, count * sizeof(float));

    // Magic immediately after the floats, then the layout signature
    const uint32_t magic = STORAGE_MAGIC;
    memcpy(buf + count * sizeof(float), &magic, sizeof(magic));
    memcpy(buf + count * sizeof(float) + sizeof(magic), &signature, sizeof(signature));

    if (!stm32_flash_erasepage(page)) {
        failed_ = true;
        return false;
    }

    const bool ok = stm32_flash_write(base, buf, buf_size);
    failed_ = !ok;
    return ok;
}

#endif // STM32H7
