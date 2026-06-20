#pragma once

//------------------------------------------------------------------------------
//   Settings for NUCLEO-H743ZI2
//------------------------------------------------------------------------------

#define HAS_PROGRAMMABLE_FDCAN_RAM_SECTIONS (true)
static const uint32_t FDCAN_MESSAGE_RAM_WORD_SIZE = 2560 ;

//------------------------------------------------------------------------------

#include <stm32h7xx_ll_rcc.h>

//------------------------------------------------------------------------------

inline uint32_t fdcanClock (void) {
  if (!__HAL_RCC_FDCAN_IS_CLK_ENABLED ()) {
  //--- Enable CAN clock
    __HAL_RCC_FDCAN_CLK_ENABLE () ;
  //--- Reset CAN peripherals
    __HAL_RCC_FDCAN_FORCE_RESET () ;
    __HAL_RCC_FDCAN_RELEASE_RESET () ;
  //--- Select CAN clock: PLL2Q (96 MHz on MicroNodePlus). PLL1Q is 137.5 MHz,
  //    which cannot divide to an exact 4 Mbps CAN-FD data phase.
    LL_RCC_SetFDCANClockSource (LL_RCC_FDCAN_CLKSOURCE_PLL2Q) ;
  }
  return LL_RCC_GetFDCANClockFreq (LL_RCC_FDCAN_CLKSOURCE) ;
}

//------------------------------------------------------------------------------
