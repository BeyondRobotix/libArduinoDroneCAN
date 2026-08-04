/*
    Host-side model of the STM32H7 flash peripheral, used to exercise the
    log-structured storage state machine in storage.cpp without hardware.

    It enforces the constraints that actually matter for the algorithm:

      * erase granularity is a whole 128 KB sector, erased state is 0xFF
      * writes must be aligned to, and a multiple of, the family's write
        unit (32 bytes on H7, 8 on L4)
      * each flash word may be programmed only ONCE per erase
        (this is the ECC constraint; stm32_flash_write_h7() enforces the
        same thing on target via stm32h7_check_all_ones())

    It also counts erases so tests can assert that the write path does not
    erase, and supports simulating power loss mid-operation.
*/

#include "flash_model.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

namespace flashmodel
{

/*
    storage.cpp reads flash through memory-mapped pointers derived from
    stm32_flash_getpageaddr(), and those addresses are uint32_t. So the model
    has to back the real 0x08000000 range rather than a heap array -- we
    reserve it from the OS at the exact address the target uses.
*/
static uint8_t *g_flash_base;

static inline uint8_t *page_ptr(uint32_t page)
{
    return g_flash_base + (size_t)page * PAGE_SIZE;
}

static void ensure_mapped()
{
    if (g_flash_base != nullptr) {
        return;
    }
    g_flash_base = (uint8_t *)VirtualAlloc((LPVOID)(uintptr_t)FLASH_BASE_ADDR,
                                           (SIZE_T)NUM_PAGES * PAGE_SIZE,
                                           MEM_RESERVE | MEM_COMMIT,
                                           PAGE_READWRITE);
    if (g_flash_base == nullptr) {
        fprintf(stderr,
                "FLASH MODEL: could not reserve 0x%08x (error %lu)\n",
                FLASH_BASE_ADDR, GetLastError());
        abort();
    }
}

static uint32_t g_erase_count;
static uint32_t g_word_writes;

// Power-loss simulation: when >= 0, the Nth flash word write fails and every
// subsequent write/erase is rejected, as if the rails had collapsed.
static long g_fail_after_writes = -1;
static bool g_powered_down;

void reset()
{
    ensure_mapped();
    memset(g_flash_base, 0xFF, (size_t)NUM_PAGES * PAGE_SIZE);
    g_erase_count = 0;
    g_word_writes = 0;
    g_fail_after_writes = -1;
    g_powered_down = false;
}

void reset_counters()
{
    g_erase_count = 0;
    g_word_writes = 0;
}

uint32_t erase_count() { return g_erase_count; }
uint32_t word_writes() { return g_word_writes; }

void arm_power_loss(long after_n_word_writes)
{
    g_fail_after_writes = after_n_word_writes;
    g_powered_down = false;
}

void restore_power()
{
    g_fail_after_writes = -1;
    g_powered_down = false;
}

uint8_t *page_data(uint32_t page)
{
    ensure_mapped();
    return page_ptr(page);
}

} // namespace flashmodel

using namespace flashmodel;

// ---- the stm32_flash_* API that storage.cpp links against -----------------

extern "C" {

uint32_t stm32_flash_getpageaddr(uint32_t page)
{
    if (page >= NUM_PAGES) {
        return 0;
    }
    return FLASH_BASE_ADDR + page * PAGE_SIZE;
}

uint32_t stm32_flash_getpagesize(uint32_t page)
{
    (void)page;
    return PAGE_SIZE;
}

uint32_t stm32_flash_getnumpages(void)
{
    return NUM_PAGES;
}

bool stm32_flash_ispageerased(uint32_t page)
{
    if (page >= NUM_PAGES) {
        return false;
    }
    ensure_mapped();
    const uint8_t *p = page_ptr(page);
    for (size_t i = 0; i < PAGE_SIZE; i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

bool stm32_flash_erasepage(uint32_t page)
{
    if (page >= NUM_PAGES || g_powered_down) {
        return false;
    }
    ensure_mapped();
    memset(page_ptr(page), 0xFF, PAGE_SIZE);
    g_erase_count++;
    return true;
}

bool stm32_flash_write(uint32_t addr, const void *buf, uint32_t count)
{
    if (g_powered_down) {
        return false;
    }
    ensure_mapped();
    // Programming granularity for the modelled family
    if ((addr % WRITE_UNIT) != 0 || (count % WRITE_UNIT) != 0) {
        fprintf(stderr, "FLASH MODEL: unaligned write addr=0x%08x count=%u\n",
                addr, count);
        abort();
    }
    if (addr < FLASH_BASE_ADDR ||
        addr + count > FLASH_BASE_ADDR + (uint32_t)(NUM_PAGES * PAGE_SIZE)) {
        fprintf(stderr, "FLASH MODEL: write out of range addr=0x%08x\n", addr);
        abort();
    }

    const uint8_t *src = (const uint8_t *)buf;
    uint32_t off = addr - FLASH_BASE_ADDR;

    for (uint32_t done = 0; done < count;
         done += WRITE_UNIT, off += WRITE_UNIT, src += WRITE_UNIT) {
        uint8_t *dst = g_flash_base + off;

        // Already holds exactly this data: the target driver skips it.
        if (memcmp(dst, src, WRITE_UNIT) == 0) {
            continue;
        }

        // A flash word can only be programmed from the erased state. On H7
        // this is the ECC constraint that rules out in-place state-machine
        // updates; modelling it everywhere keeps one shared rule.
        for (size_t i = 0; i < WRITE_UNIT; i++) {
            if (dst[i] != 0xFF) {
                fprintf(stderr,
                        "FLASH MODEL: rewrite of programmed word at 0x%08x "
                        "(ECC violation)\n",
                        FLASH_BASE_ADDR + off);
                abort();
            }
        }

        if (g_fail_after_writes >= 0 && (long)g_word_writes >= g_fail_after_writes) {
            g_powered_down = true;
            return false;
        }

        memcpy(dst, src, WRITE_UNIT);
        g_word_writes++;
    }
    return true;
}

} // extern "C"
