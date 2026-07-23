#pragma once

#include <stdint.h>
#include <stddef.h>

namespace flashmodel
{

// Flash geometry, selected by the build so one model serves both families.
//   MODEL_H7 : STM32H723 -- 1 MB as 8 x 128 KB sectors, 32-byte write unit
//   otherwise: STM32L431 -- 256 KB as 128 x 2 KB pages,  8-byte write unit
constexpr uint32_t FLASH_BASE_ADDR = 0x08000000u;
#if defined(MODEL_H7)
constexpr size_t   PAGE_SIZE  = 128u * 1024u;
constexpr uint32_t NUM_PAGES  = 8u;
constexpr size_t   WRITE_UNIT = 32u;
#else
constexpr size_t   PAGE_SIZE  = 2u * 1024u;
constexpr uint32_t NUM_PAGES  = 128u;
constexpr size_t   WRITE_UNIT = 8u;
#endif

// Wipe the whole model to the erased state and clear all counters.
void reset();

// Zero the erase / write counters without touching flash contents.
void reset_counters();

uint32_t erase_count();
uint32_t word_writes();

// Simulate power loss: the Nth (0-based) flash word write from now on fails,
// and every write/erase after it is rejected. Use restore_power() to model
// the subsequent reboot.
void arm_power_loss(long after_n_word_writes);
void restore_power();

uint8_t *page_data(uint32_t page);

} // namespace flashmodel
