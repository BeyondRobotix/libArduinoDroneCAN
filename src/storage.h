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
    a given key wins. When the active page fills, the live values are
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

    ---- Instances -----------------------------------------------------------

    One page pair serves every DroneCAN instance on the board. Each record
    carries the instance it belongs to, so a dual-CAN node does NOT need a
    second pair of flash pages -- which on H7 would mean another 256 KB.

    Because two instances can have different parameter lists, the layout
    signature cannot live in the sector header. Each instance instead appends
    a *layout record* (its signature and parameter count) the first time it
    writes. Replay tracks validity per instance as it scans, so a firmware
    change to one instance's parameter list invalidates only that instance's
    earlier records and leaves the other's alone.

    ---- Areas ---------------------------------------------------------------

    Records also carry a 4-bit area, of which only AREA_PARAM is implemented.
    The remaining values are reserved so bulk storage (mission data and
    similar) can be layered on later without another on-flash format change.
    See the note in storage.cpp before using one.
*/
class DroneCAN_Storage
{
public:
    // Return the first flash page of the storage pair. The pair is
    // (page, page+1) and is shared by every instance.
    static uint32_t default_page();

    // Load parameter values for `instance` into `values`.
    //
    // `signature` must be built with signature_begin()/signature_feed() over
    // that instance's parameter name list. Returns true only if storage held
    // values written under the same signature and count; `values` is left
    // untouched otherwise, so the caller's code defaults survive. Indices
    // never written also keep their incoming value.
    //
    // This is also the recovery point: it resolves which page is active and
    // pays any erase the previous power cycle deferred. Call it once per
    // instance at startup, before any save() for that instance.
    static bool load(float *values, size_t count, uint32_t page,
                     uint8_t instance, uint32_t signature);

    // Persist a single parameter value at `index`. Returns false on failure.
    static bool save(size_t index, float value, size_t total_count,
                     uint32_t page, uint8_t instance, uint32_t signature);

    // Persist all parameter values at once. Returns false on failure.
    static bool save_all(const float *values, size_t count, uint32_t page,
                         uint8_t instance, uint32_t signature);

    // True until a flash write or erase fails; latched until the next
    // successful load(). Callers can surface this via NodeStatus health.
    static bool healthy();

    // Build a signature over the parameter name list:
    //     uint32_t sig = signature_begin();
    //     for (auto &p : parameters) sig = signature_feed(sig, p.name);
    static uint32_t signature_begin();
    static uint32_t signature_feed(uint32_t sig, const char *name);

    // Largest parameter set per instance. Bounds the compaction buffer, and
    // the written-index mask below is sized to it. Override with
    // -DSTORAGE_MAX_PARAMS=N (max 64: the mask is a uint64_t).
#ifndef STORAGE_MAX_PARAMS
    static constexpr size_t MAX_PARAMS = 64;
#else
    static constexpr size_t MAX_PARAMS = STORAGE_MAX_PARAMS;
#endif
    static_assert(MAX_PARAMS <= 64, "written-index mask is a uint64_t");

    // Instances sharing the page pair. 2 covers PORT1 + PORT2; the record
    // field is 4 bits so this can grow to 16 if a board ever needs it.
#ifndef STORAGE_MAX_INSTANCES
    static constexpr size_t MAX_INSTANCES = 2;
#else
    static constexpr size_t MAX_INSTANCES = STORAGE_MAX_INSTANCES;
#endif
    static_assert(MAX_INSTANCES <= 16, "instance field is 4 bits");

private:
    // Latched failure flag behind healthy().
    static bool failed_;
};

#endif // DRONECAN_STORAGE_H
