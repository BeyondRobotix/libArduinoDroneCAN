#include "storage.h"
#include "flash.h"
#include <Arduino.h>
#include <string.h>

/*
    Return the flash page for a given port index (0=PORT1, 1=PORT2).
    PORT1 gets the last page; PORT2 gets second-to-last.
    STORAGE_FLASH_PAGE overrides everything for single-instance builds.
*/
uint32_t DroneCAN_Storage::default_page(uint8_t port_index)
{
#ifdef STORAGE_FLASH_PAGE
    return STORAGE_FLASH_PAGE;
#else
    uint32_t last = stm32_flash_getnumpages() - 1;
    return (port_index <= 1) ? (last - port_index) : last;
#endif
}

/*
    Round n up to the next WRITE_ALIGN boundary.
*/
size_t DroneCAN_Storage::align_up(size_t n)
{
    return (n + WRITE_ALIGN - 1) & ~(WRITE_ALIGN - 1);
}

/*
    Load parameter values by reading directly from the memory-mapped flash.
    Returns false (and leaves `values` untouched) when the magic marker is
    missing — this preserves code-defined defaults on first boot or after
    a full chip erase.
*/
bool DroneCAN_Storage::load(float *values, size_t count, uint32_t page)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }

    // On H7 the DCache can serve stale data for memory-mapped flash.
    // Invalidate the region we are about to read so we see what is
    // actually in flash.
    size_t region = align_up(count * sizeof(float) + sizeof(uint32_t));
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    SCB_InvalidateDCache_by_Addr((void *)base, region);
#endif

    // The magic word sits right after the parameter floats
    size_t magic_offset = count * sizeof(float);
    uint32_t magic;
    memcpy(&magic, (const void *)(base + magic_offset), sizeof(magic));
    if (magic != STORAGE_MAGIC) {
        return false;
    }

    // Read the parameter values straight from flash
    memcpy(values, (const void *)base, count * sizeof(float));
    return true;
}

/*
    Persist a single parameter.  Because flash pages must be erased as a
    whole before any byte can be rewritten, we:
      1. read the current blob from flash into RAM
      2. patch the one value that changed
      3. erase + rewrite the full page
*/
void DroneCAN_Storage::save(size_t index, float value, size_t total_count, uint32_t page)
{
    // Build a working copy of all values
    float buf[total_count];
    uint32_t base = stm32_flash_getpageaddr(page);

    // Start from flash if it has valid data, otherwise zero-init
    size_t magic_offset = total_count * sizeof(float);
    uint32_t magic;
    memcpy(&magic, (const void *)(base + magic_offset), sizeof(magic));
    if (magic == STORAGE_MAGIC) {
        memcpy(buf, (const void *)base, total_count * sizeof(float));
    } else {
        memset(buf, 0, sizeof(buf));
    }

    buf[index] = value;
    write_page(buf, total_count, page);
}

/*
    Persist every parameter value.
*/
void DroneCAN_Storage::save_all(const float *values, size_t count, uint32_t page)
{
    write_page(values, count, page);
}

/*
    Erase the storage page and write the parameter blob + magic marker.
    The buffer written to flash is zero-padded to WRITE_ALIGN so that the
    H7's 32-byte write requirement is always satisfied.
*/
bool DroneCAN_Storage::write_page(const float *values, size_t count, uint32_t page)
{
    uint32_t base = stm32_flash_getpageaddr(page);
    if (base == 0) {
        return false;
    }

    // Total payload: N floats + 1 magic uint32_t, rounded up
    size_t raw_size = count * sizeof(float) + sizeof(uint32_t);
    size_t buf_size = align_up(raw_size);

    // Build an aligned write buffer (zero-padded)
    uint8_t buf[buf_size];
    memset(buf, 0, buf_size);
    memcpy(buf, values, count * sizeof(float));

    // Place magic immediately after the floats
    uint32_t magic = STORAGE_MAGIC;
    memcpy(buf + count * sizeof(float), &magic, sizeof(magic));

    // Erase the page
    if (!stm32_flash_erasepage(page)) {
        return false;
    }

    // Write the blob
    return stm32_flash_write(base, buf, buf_size);
}
