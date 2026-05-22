#ifdef ARDUINO_NUCLEO_H723ZG
#include "Arduino.h"
#include "canH723.h"

// Message RAM split (must be defined before including ACANFD_STM32.h).
// H723 has 2560 words total shared across FDCAN1/2/3. Equal 3-way split;
// FDCAN3 is reserved for future use (not routed on MicroNodePlus).
#define FDCAN1_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN2_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN3_MESSAGE_RAM_WORD_SIZE 800

#include <ACANFD_STM32.h>

// Per-port driver state. Index 0 = PORT1 (FDCAN1), index 1 = PORT2 (FDCAN2).
struct PortState {
    ACANFD_STM32 *drv          = nullptr;
    uint32_t      begin_status = 0xFFFF;
    bool          initialized  = false;
};
static PortState gPorts[2];

// Map port index to driver instance + pin pair.
// PORT1: FDCAN1 at PD_0 (RX) / PD_1 (TX) — today's production wiring on MicroNodePlus.
// PORT2: FDCAN2 at PB_5 (RX) / PB_6 (TX) — placeholder; confirm from board schematic.
struct PortConfig {
    ACANFD_STM32 *drv;
    uint32_t      rx_pin;
    uint32_t      tx_pin;
};
static const PortConfig kPortConfig[2] = {
    { &fdcan1, PD_0, PD_1 }, // PORT1
    { &fdcan2, PB_5, PB_6 }, // PORT2 (placeholder pins — verify with schematic)
};

static uint32_t bitrate_to_hz(BITRATE bitrate) {
    switch (bitrate) {
        case CAN_50KBPS:   return  50000;
        case CAN_100KBPS:  return 100000;
        case CAN_125KBPS:  return 125000;
        case CAN_250KBPS:  return 250000;
        case CAN_500KBPS:  return 500000;
        case CAN_1000KBPS: return 1000000;
    }
    return 1000000;
}

static bool init_port(uint8_t idx, ACANFD_STM32_Settings &settings) {
    settings.mRxPin = kPortConfig[idx].rx_pin;
    settings.mTxPin = kPortConfig[idx].tx_pin;
    gPorts[idx].drv = kPortConfig[idx].drv;
    gPorts[idx].begin_status = gPorts[idx].drv->beginFD(settings);
    gPorts[idx].initialized  = (gPorts[idx].begin_status == 0);
    return gPorts[idx].initialized;
}

bool CANInit(BITRATE bitrate, uint8_t port) {
    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor::x1);
    if (port == CAN_PORT_BOTH) {
        return init_port(0, settings) & init_port(1, settings);
    }
    if (port > 1) return false;
    return init_port(port, settings);
}

bool CANInit_fd(BITRATE bitrate, uint8_t port) {
    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor::x4);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
    if (port == CAN_PORT_BOTH) {
        return init_port(0, settings) & init_port(1, settings);
    }
    if (port > 1) return false;
    return init_port(port, settings);
}

static void send_on(PortState &p, const CanardCANFrame *tx_msg) {
    if (!p.initialized || !p.drv || !tx_msg) return;
    CANFDMessage message;
    message.ext = true;
    message.id  = tx_msg->id & 0x1FFFFFFFU;
    message.len = tx_msg->data_len;
    memcpy(message.data, tx_msg->data, tx_msg->data_len);
#if CANARD_ENABLE_CANFD
    message.type = tx_msg->canfd
                 ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                 : CANFDMessage::CAN_DATA;
#else
    message.type = CANFDMessage::CAN_DATA;
#endif
    p.drv->tryToSendReturnStatusFD(message);
}

void CANSend(const CanardCANFrame *tx_msg, uint8_t port) {
    if (port == CAN_PORT_BOTH) {
        send_on(gPorts[0], tx_msg);
        send_on(gPorts[1], tx_msg);
    } else if (port <= 1) {
        send_on(gPorts[port], tx_msg);
    }
}

static bool recv_from(PortState &p, CanardCANFrame *rx_msg) {
    if (!p.initialized || !p.drv || !p.drv->availableFD0()) return false;
    CANFDMessage message;
    if (!p.drv->receiveFD0(message)) return false;
    if (!message.ext) return false;
    rx_msg->id       = (message.id & 0x1FFFFFFFU) | CANARD_CAN_FRAME_EFF;
    rx_msg->data_len = message.len;
    memcpy(rx_msg->data, message.data, message.len);
#if CANARD_ENABLE_CANFD
    rx_msg->canfd = (message.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                  || message.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH);
#endif
    return true;
}

static uint8_t gRxNext = 0;

void CANReceive(CanardCANFrame *rx_msg, uint8_t port) {
    if (!rx_msg) return;
    rx_msg->id = 0;
    if (port <= 1) {
        recv_from(gPorts[port], rx_msg);
    } else {
        if (!recv_from(gPorts[gRxNext], rx_msg)) {
            gRxNext ^= 1;
            recv_from(gPorts[gRxNext], rx_msg);
        } else {
            gRxNext ^= 1;
        }
    }
}

uint8_t CANMsgAvail(uint8_t port) {
    if (port == CAN_PORT_BOTH) {
        uint8_t n = 0;
        if (gPorts[0].initialized && gPorts[0].drv) n += gPorts[0].drv->availableFD0();
        if (gPorts[1].initialized && gPorts[1].drv) n += gPorts[1].drv->availableFD0();
        return n;
    }
    if (port > 1 || !gPorts[port].initialized || !gPorts[port].drv) return 0;
    return gPorts[port].drv->availableFD0();
}

#endif // ARDUINO_NUCLEO_H723ZG
