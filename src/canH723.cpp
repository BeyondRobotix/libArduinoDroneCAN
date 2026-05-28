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
    { &fdcan2, PB_5, PB_6 }, // PORT2 — FDCAN2_RX=PB5, FDCAN2_TX=PB6 (AF9)
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

// "Classic" mode: peripheral runs NORMAL_FD at x4 data rate so it can *receive*
// FD frames from other nodes (e.g. ArduPilot with UC_OPTION +4). The Classic/FD
// distinction only affects the per-frame TX canfd flag, not the peripheral config.
bool CANInit(BITRATE bitrate, uint8_t port) {
    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor::x4);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
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

// Returns true if the message was accepted by the driver's TX FIFO.
static bool send_on(PortState &p, const CanardCANFrame *tx_msg) {
    if (!p.initialized || !p.drv || !tx_msg) return false;
    CANFDMessage message;
    message.ext = true;
    message.id  = tx_msg->id & 0x1FFFFFFFU;
    message.len = tx_msg->data_len;
    memcpy(message.data, tx_msg->data, tx_msg->data_len);
#if CANARD_ENABLE_CANFD
    // Temporarily forcing NO_BIT_RATE_SWITCH while diagnosing the 4 Mbps BRS
    // bus-off issue. FD framing intact, payload stays at nominal bitrate.
    // Restore to CANFD_WITH_BIT_RATE_SWITCH once the BRS issue is resolved.
    message.type = tx_msg->canfd
                 ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                 : CANFDMessage::CAN_DATA;
#else
    message.type = CANFDMessage::CAN_DATA;
#endif
    // tryToSendReturnStatusFD returns 0 on success, non-zero on FIFO-full / error.
    return p.drv->tryToSendReturnStatusFD(message) == 0;
}

bool CANSend(const CanardCANFrame *tx_msg, uint8_t port) {
    if (port == CAN_PORT_BOTH) {
        // For the fan-out case, succeed only if both peripherals accept the frame.
        // If either fails the caller will retry next cycle.
        const bool a = send_on(gPorts[0], tx_msg);
        const bool b = send_on(gPorts[1], tx_msg);
        return a && b;
    }
    if (port <= 1) {
        return send_on(gPorts[port], tx_msg);
    }
    return false;
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

// Edge-triggered bus-off detector. Called from CANMsgAvail() so it's polled
// every main-loop iteration. ACANFD_STM32::statusFlags() bit 4 = PSR.BO.
// Prints once when a port enters bus-off and once when it recovers — no spam.
static void check_bus_off_once(uint8_t idx) {
    static bool was_bus_off[2] = { false, false };
    if (!gPorts[idx].initialized || !gPorts[idx].drv) return;
    const bool bus_off = (gPorts[idx].drv->statusFlags() & (1U << 4)) != 0;
    if (bus_off != was_bus_off[idx]) {
        if (Serial) {
            Serial.print(bus_off ? "CANFD BUS-OFF on PORT" : "CANFD recovered on PORT");
            Serial.println(idx + 1);
        }
        was_bus_off[idx] = bus_off;
    }
}

uint8_t CANMsgAvail(uint8_t port) {
    if (port == CAN_PORT_BOTH) {
        check_bus_off_once(0);
        check_bus_off_once(1);
        uint8_t n = 0;
        if (gPorts[0].initialized && gPorts[0].drv) n += gPorts[0].drv->availableFD0();
        if (gPorts[1].initialized && gPorts[1].drv) n += gPorts[1].drv->availableFD0();
        return n;
    }
    if (port > 1 || !gPorts[port].initialized || !gPorts[port].drv) return 0;
    check_bus_off_once(port);
    return gPorts[port].drv->availableFD0();
}

#endif // ARDUINO_NUCLEO_H723ZG
