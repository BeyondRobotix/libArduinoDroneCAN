#ifndef DRONECAN_STORAGE_H
#define DRONECAN_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/*
    Flash-based parameter storage.

    Two implementations are selected at compile time, matching the way
    flash.cpp already dispatches on MCU family:

    ---- H7 (CoreNode H743, MicroNodePlus H723): log-structured ----------------

    The H7 erase unit is a 128 KB sector and an erase blocks the core for
    roughly a second with interrupts disabled -- far too long to sit in the
    path of a parameter save. So the H7 build never erases on the write path.

    Two sectors are used as a ping-pong pair (`page` and `page - 1`). The
    active sector holds a header followed by an append-only log of 32-byte
    records, one per parameter write. Loading replays the log; the last
    record for a given index wins. When the active sector fills, the values
    are compacted into the spare sector and the old one is marked FULL --
    it is erased at the next load(), not inline.

    Records are 32 bytes because that is the H7 flash word: a 256-bit write
    is the smallest unit the peripheral accepts, and ECC is computed over
    the whole word, so a word can be programmed exactly once per erase.
    That also rules out the progressive bit-clearing state machine used on
    other families -- each sector state transition claims its own fresh
    32-byte slot instead (see `sector_slot`).

    ---- L4 / G4 / F4: erase-and-rewrite (unchanged behaviour) ----------------

    The erase unit is 2 KB and an erase takes ~20 ms, so the simple scheme
    is fine. One page holds the whole parameter blob:

        [float param_0] ... [float param_N-1] [uint32_t MAGIC] [uint32_t SIG]

    padded up to WRITE_ALIGN. Every save erases the page and rewrites it.

    --------------------------------------------------------------------------

    Both implementations store a signature derived from the parameter name
    list. A firmware update that reorders or renames parameters changes the
    signature, so stored values are rejected and the code defaults are used
    instead of being silently reassigned to the wrong parameters.

    STORAGE_FLASH_PAGE can be defined at build time to override the default
    page (last page, or last sector of the pair on H7).
*/
class DroneCAN_Storage
{
public:
    // Return the flash page number for a given port index (0=PORT1, 1=PORT2).
    // On H7 each port owns a two-sector pair and this returns the first of
    // the pair (the second is `page - 1`). Elsewhere each port owns one page.
    static uint32_t default_page(uint8_t port_index = 0);

    // Load parameter values from storage into `values`.
    //
    // `signature` must be built with signature_begin()/signature_feed() over
    // the current parameter name list. Returns true only if storage held
    // valid data written under the same signature; `values` is left untouched
    // otherwise, so the caller's code defaults survive.
    static bool load(float *values, size_t count, uint32_t page, uint32_t signature);

    // Persist a single parameter value at `index`. Returns false on failure.
    static bool save(size_t index, float value, size_t total_count,
                     uint32_t page, uint32_t signature);

    // Persist all parameter values at once. Returns false on failure.
    static bool save_all(const float *values, size_t count,
                         uint32_t page, uint32_t signature);

    // True until a flash write or erase fails; latched until the next
    // successful load(). Callers can surface this via NodeStatus health.
    static bool healthy();

    // Build a signature over the parameter name list:
    //     uint32_t sig = signature_begin();
    //     for (auto &p : parameters) sig = signature_feed(sig, p.name);
    // Also folds in the parameter count, so add/remove is caught too.
    static uint32_t signature_begin();
    static uint32_t signature_feed(uint32_t sig, const char *name);

    // Largest parameter set the storage layer will handle. Bounds the
    // stack buffers used when rebuilding the value set (previously VLAs
    // sized from a runtime count). Override with -DSTORAGE_MAX_PARAMS=N.
#ifndef STORAGE_MAX_PARAMS
    static constexpr size_t MAX_PARAMS = 64;
#else
    static constexpr size_t MAX_PARAMS = STORAGE_MAX_PARAMS;
#endif

private:
    static constexpr uint32_t STORAGE_MAGIC = 0x4443414E; // "DCAN"

    // Minimum write granularity. H7 needs 32 (256-bit flash word); F4 is 2
    // and G4/L4 are 8. The non-H7 path rounds up to 32 for all of them --
    // it costs a few bytes of a 2 KB page and keeps one code path.
    static constexpr size_t WRITE_ALIGN = 32;

    // Latched failure flag behind healthy().
    static bool failed_;

    // Round a byte count up to the next WRITE_ALIGN boundary.
    static size_t align_up(size_t n);

    // Same, usable as an array bound.
    static constexpr size_t align_up_const(size_t n)
    {
        return (n + WRITE_ALIGN - 1) & ~(WRITE_ALIGN - 1);
    }

#if !defined(STM32H7) && !defined(STM32H7xx)
    // Non-H7: write the full blob (params + magic + signature) to the page.
    // Erases first, then writes. Returns true on success.
    static bool write_page(const float *values, size_t count,
                           uint32_t page, uint32_t signature);
#endif
};

#endif // DRONECAN_STORAGE_H
