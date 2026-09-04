#ifdef CANC532
#ifndef CAN_DRIVER_C5_
#define CAN_DRIVER_C5_

/*
 * UNVERIFIED -- STM32C532 (MicroNodeC5) FDCAN driver.
 *
 * Written directly against STM32C5's "HAL2" FDCAN API (hal_status_t /
 * hal_fdcan_handle_t / HAL_FDCAN_*), a different generation from the
 * classic HAL used by canH743.cpp/canH723.cpp. The third-party acanfd-stm32
 * library those drivers wrap does not support this HAL generation at all
 * (its per-board settings/objects headers call classic HAL/LL FDCAN macros
 * that don't exist for STM32C5), so this driver talks to the HAL2 API
 * directly instead of going through acanfd-stm32.
 *
 * Scope: classic CAN 2.0B only (no CAN-FD) -- matches CANL431's scope, see
 * the #error guard in dronecan.h. STM32C532 has a single FDCAN instance, so
 * only port 0 (CAN_PORT1) is valid; other port values fail.
 *
 * Has not been build- or hardware-tested (no PlatformIO registry access was
 * available in the environment this was written in). Before trusting this
 * on a real bus: verify the nominal bit timing math against an
 * oscilloscope/CAN analyzer, and confirm PA11/PA12 AF9 + PCLK1-as-kernel-
 * clock against the actual board schematic once one exists.
 */

#include <canard.h>

enum BITRATE
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};

// STM32C532 has one FDCAN instance. Port argument kept for call-site
// compatibility with the H7 drivers (dronecan.cpp's generic dispatch path
// calls CANInit(bitrate, port) uniformly); only CAN_PORT1 is valid here.
static constexpr uint8_t CAN_PORT1     = 0;
static constexpr uint8_t CAN_PORT2     = 1; // not present on this chip
static constexpr uint8_t CAN_PORT_BOTH = 2; // not present on this chip

bool    CANInit(BITRATE bitrate, uint8_t port = CAN_PORT1);
bool    CANSend(const CanardCANFrame *tx_msg, uint8_t port = CAN_PORT1);
void    CANReceive(CanardCANFrame *rx_msg, uint8_t port = CAN_PORT1);
uint8_t CANMsgAvail(uint8_t port = CAN_PORT1);

#endif // CAN_DRIVER_C5_
#endif // CANC532
