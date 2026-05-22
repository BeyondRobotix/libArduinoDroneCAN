#ifdef CANH7
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
// PORT1 on CoreNode = FDCAN2 (today's wiring, PB_5/PB_6).
// PORT2 on CoreNode = FDCAN1 (placeholder pins PB_8/PB_9 — confirm from schematic).
static constexpr uint8_t CAN_PORT1 = 0;
static constexpr uint8_t CAN_PORT2 = 1;
static constexpr uint8_t CAN_PORT_BOTH = 2;

// Classic CAN init for the given port.
bool CANInit(BITRATE bitrate, uint8_t port = CAN_PORT1);

// CAN-FD init (NORMAL_FD, 4x data rate) for the given port.
bool CANInit_fd(BITRATE bitrate, uint8_t port = CAN_PORT1);

// Send a frame on the given port. BOTH fans out to both peripherals.
void CANSend(const CanardCANFrame *tx_msg, uint8_t port = CAN_PORT1);

// Receive one frame from the given port. BOTH drains whichever port has data.
void CANReceive(CanardCANFrame *rx_msg, uint8_t port = CAN_PORT1);

// Number of frames pending in the RX FIFO for the given port (BOTH = sum).
uint8_t CANMsgAvail(uint8_t port = CAN_PORT1);

#endif // CAN_DRIVER_H7_
#endif // CANH7
