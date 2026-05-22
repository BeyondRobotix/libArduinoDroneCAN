#ifdef ARDUINO_NUCLEO_H723ZG
#ifndef CAN_DRIVER_H7_
#define CAN_DRIVER_H7_

#include <canard.h>
#include <ACANFD_STM32_from_cpp.h>

enum BITRATE
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};

// Port selectors — values match DroneCAN::CanPort (0=PORT1, 1=PORT2, 2=BOTH).
// PORT1 on MicroNodePlus = FDCAN1 (today's wiring, PD_0/PD_1).
// PORT2 on MicroNodePlus = FDCAN2 (placeholder pins PB_5/PB_6 — confirm from schematic).
static constexpr uint8_t CAN_PORT1      = 0;
static constexpr uint8_t CAN_PORT2      = 1;
static constexpr uint8_t CAN_PORT_BOTH  = 2;

bool    CANInit(BITRATE bitrate, uint8_t port = CAN_PORT1);
bool    CANInit_fd(BITRATE bitrate, uint8_t port = CAN_PORT1);
void    CANSend(const CanardCANFrame *tx_msg, uint8_t port = CAN_PORT1);
void    CANReceive(CanardCANFrame *rx_msg, uint8_t port = CAN_PORT1);
uint8_t CANMsgAvail(uint8_t port = CAN_PORT1);

#endif // CAN_DRIVER_H7_
#endif // ARDUINO_NUCLEO_H723ZG
