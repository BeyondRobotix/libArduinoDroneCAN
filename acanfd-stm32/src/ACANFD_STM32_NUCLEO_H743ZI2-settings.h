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
  //--- Select CAN clock: PLL2Q (96 MHz on CoreNode). 96 MHz divides into
  //    1/2/4/8 Mbps with prescaler 1, which CAN-FD's TDC needs; PLL1Q (120 MHz)
  //    forces BRP=2 at 2 Mbps. Matches MicroNodePlus (also PLL2Q).
    LL_RCC_SetFDCANClockSource (LL_RCC_FDCAN_CLKSOURCE_PLL2Q) ;
  }
  return LL_RCC_GetFDCANClockFreq (LL_RCC_FDCAN_CLKSOURCE) ;
}

//------------------------------------------------------------------------------
