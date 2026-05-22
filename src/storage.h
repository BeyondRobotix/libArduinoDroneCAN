#ifndef DRONECAN_STORAGE_H
#define DRONECAN_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/*
    Flash-based parameter storage.

    Uses the last page of internal flash. The layout is:

        [float param_0] [float param_1] ... [float param_N-1] [uint32_t MAGIC]

    padded to the flash write alignment of the target MCU (32 bytes on H7).
    Because flash can only be written once after an erase, every save
    erases the page and rewrites the full parameter set.

    STORAGE_FLASH_PAGE can be defined at build time to override the default
    (last page).
*/
class DroneCAN_Storage
{
public:
    // Return the flash page number for a given port index (0=PORT1, 1=PORT2).
    // PORT1 -> last page, PORT2 -> second-to-last.
    // Callers may pass a specific page override instead of calling this.
    static uint32_t default_page(uint8_t port_index = 0);

    // Load parameter values from the given flash page into the array.
    // Returns true if the page contained valid data; leaves array untouched otherwise.
    static bool load(float *values, size_t count, uint32_t page);

    // Persist a single parameter value at the given index on the given page.
    static void save(size_t index, float value, size_t total_count, uint32_t page);

    // Persist all parameter values at once to the given page.
    static void save_all(const float *values, size_t count, uint32_t page);

private:
    static constexpr uint32_t STORAGE_MAGIC = 0x4443414E; // "DCAN"

    // Minimum write granularity — must match the target MCU.
    // H7 = 32 bytes, F4 = 2 bytes, G4/L4 = 8 bytes.  We round up
    // to the largest so the same logic works everywhere.
    static constexpr size_t WRITE_ALIGN = 32;

    // Round a byte count up to the next WRITE_ALIGN boundary.
    static size_t align_up(size_t n);

    // Write the full blob (params + magic) to the given page.
    // Erases first, then writes.  Returns true on success.
    static bool write_page(const float *values, size_t count, uint32_t page);
};

#endif // DRONECAN_STORAGE_H
