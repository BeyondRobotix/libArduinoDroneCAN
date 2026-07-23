#ifndef DRONECAN_STORAGE_H
#define DRONECAN_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/*
    Log-structured flash parameter storage.

    Persisting a parameter must not erase flash. An erase blocks the core
    with interrupts disabled -- ~1 s for a 128 KB H7 sector, ~20 ms for a
    2 KB L4 page -- and, more importantly, flash endurance is only ~10k
    erase cycles per page. A design that erases per write puts a hard
    ceiling on how many times a parameter can ever be written.

    So instead of rewriting a fixed blob in place, this keeps an append-only
    log across a ping-pong pair of pages, following ArduPilot's
    AP_FlashStorage:

        [ sector header: 3 state slots ][ record ][ record ][ ... ]

    A save appends one record. Loading replays the log; the last record for
    a given index wins. When the active page fills, the live values are
    compacted into the pre-erased spare and the old page is marked FULL --
    its erase is deferred to the next load(), i.e. to boot. The write path
    never erases.

    Records per page, and therefore writes per erase cycle:

        L4  (2 KB page,      8-byte records) ->  250
        H7  (128 KB sector, 32-byte records) -> 4093

    Record size is the family's flash programming granularity. On H7 that
    is a 256-bit flash word: ECC is computed across the whole word, so a
    word can be programmed exactly once per erase. That rules out the
    progressive bit-clearing state machine other families allow, so each
    sector state transition claims its own fresh slot rather than rewriting
    one in place. The same slot scheme is used everywhere for one code path.

    Every entry carries a signature derived from the parameter name list. A
    firmware update that renames, reorders or resizes the parameter set
    changes the signature, so stale values are rejected and the code
    defaults are used instead of being silently reassigned to the wrong
    parameters.

    The page pair is chosen by default_page(): PORT1 takes (base, base+1)
    and PORT2 takes (base+2, base+3), where base is STORAGE_FLASH_PAGE from
    the board definition. Boards place that region deliberately -- on L4 in
    the tail of the bootloader reserve, on H7 at the top of flash -- and the
    linker script reserves it. See br_platformio_hwdef.
*/
class DroneCAN_Storage
{
public:
    // Return the first flash page of the pair used by the given port index
    // (0=PORT1, 1=PORT2). The pair is (page, page+1).
    static uint32_t default_page(uint8_t port_index = 0);

    // Load parameter values from storage into `values`.
    //
    // `signature` must be built with signature_begin()/signature_feed() over
    // the current parameter name list. Returns true only if storage held
    // valid data written under the same signature; `values` is left
    // untouched otherwise, so the caller's code defaults survive.
    //
    // This is also the recovery point: it resolves which page is active and
    // pays any erase the previous power cycle deferred. Call it once at
    // startup before any save().
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
    static uint32_t signature_begin();
    static uint32_t signature_feed(uint32_t sig, const char *name);

    // Largest parameter set the storage layer will handle. Bounds the stack
    // buffer used when compacting. Override with -DSTORAGE_MAX_PARAMS=N.
#ifndef STORAGE_MAX_PARAMS
    static constexpr size_t MAX_PARAMS = 64;
#else
    static constexpr size_t MAX_PARAMS = STORAGE_MAX_PARAMS;
#endif

private:
    // Latched failure flag behind healthy().
    static bool failed_;
};

#endif // DRONECAN_STORAGE_H
