#ifdef CANH7
#include "Arduino.h"
#include "canH743.h"

// Message RAM split (must be defined before including ACANFD_STM32.h).
// H743 has 2560 words total. Split 50/50 so either port can be enabled independently
// or both run concurrently. A PORT1-only build wastes the unused 1280 words — acceptable.
#define FDCAN1_MESSAGE_RAM_WORD_SIZE 1280
#define FDCAN2_MESSAGE_RAM_WORD_SIZE 1280

#include <ACANFD_STM32.h>

#define CAN_EXT_ID_MASK 0x1FFFFFFFU

// Per-port driver state. Index 0 = PORT1 (FDCAN1 = physical CAN1),
// index 1 = PORT2 (FDCAN2 = physical CAN2). Matches MicroNodePlus, where
// PORT1 is also FDCAN1.
struct PortState {
    ACANFD_STM32 *drv          = nullptr;
    uint32_t      begin_status = 0xFFFF;
    bool          initialized  = false;
};
static PortState gPorts[2];

// Map port index to driver instance + pin pair. Port number tracks the silk
// label: PORT1 = CoreNode connector "CAN1", PORT2 = "CAN2".
// PORT1: FDCAN1 at PH_14 (RX) / PD_1 (TX) — physical CAN1 connector.
//   (Earlier PB_8/PB_9 placeholders were wrong — FDCAN1 routes to PD1/PH14 here,
//    verified on hardware; PH_14 was added to fdcan1_rx_pin_array in the H743
//    objects header.)
// PORT2: FDCAN2 at PB_5 (RX) / PB_6 (TX) — physical CAN2 connector.
struct PortConfig {
    ACANFD_STM32 *drv;
    uint32_t      rx_pin;
    uint32_t      tx_pin;
};
static const PortConfig kPortConfig[2] = {
    { &fdcan1, PH_14, PD_1 }, // PORT1 — physical CAN1 (FDCAN1)
    { &fdcan2, PB_5,  PB_6 }, // PORT2 — physical CAN2 (FDCAN2)
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

// Wrapper-level begin_status: the FDCAN kernel clock cannot hit the requested
// bitrates within tolerance (settings.mBitSettingOk). Distinct from the
// driver's beginFD() error flags, which never reach bit 31.
static const uint32_t kBitTimingOutOfTolerance = 1UL << 31;

static bool init_port(uint8_t idx, ACANFD_STM32_Settings &settings) {
    gPorts[idx].drv = kPortConfig[idx].drv;
    if (!settings.mBitSettingOk) {
        // Refuse to start with off-frequency bit timing: it passes the driver's
        // range checks but produces bit errors / bus-off on a real bus.
        gPorts[idx].begin_status = kBitTimingOutOfTolerance;
        gPorts[idx].initialized  = false;
        return false;
    }
    settings.mRxPin = kPortConfig[idx].rx_pin;
    settings.mTxPin = kPortConfig[idx].tx_pin;
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

bool CANInit_fd(BITRATE bitrate, uint8_t port, uint8_t data_factor) {
    if (data_factor < 1 || data_factor > 10) data_factor = 4;
    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor(data_factor));
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
    message.id  = tx_msg->id & CAN_EXT_ID_MASK;
    message.len = tx_msg->data_len;
    memcpy(message.data, tx_msg->data, tx_msg->data_len);
#if CANARD_ENABLE_CANFD
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
    rx_msg->id       = message.id | CANARD_CAN_FRAME_EFF;
    rx_msg->data_len = message.len;
    memcpy(rx_msg->data, message.data, message.len);
#if CANARD_ENABLE_CANFD
    rx_msg->canfd = (message.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                  || message.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH);
#endif
    return true;
}

// Round-robin state for BOTH-mode RX fairness.
static uint8_t gRxNext = 0;

void CANReceive(CanardCANFrame *rx_msg, uint8_t port) {
    if (!rx_msg) return;
    rx_msg->id = 0;
    if (port <= 1) {
        recv_from(gPorts[port], rx_msg);
    } else {
        // Try the next port first, then the other, to keep both draining fairly.
        if (!recv_from(gPorts[gRxNext], rx_msg)) {
            gRxNext ^= 1;
            recv_from(gPorts[gRxNext], rx_msg);
        } else {
            gRxNext ^= 1;
        }
    }
}

// Edge-triggered bus-off detector + recovery. Called from CANMsgAvail() so it's
// polled every main-loop iteration. ACANFD_STM32::statusFlags() bit 4 = PSR.BO.
// Prints once when a port enters bus-off and once when it recovers — no spam.
// M_CAN parks in INIT after bus-off and stays off the bus until software
// restarts it; retry recovery at 1 Hz so a transient bus fault doesn't take the
// node offline until reboot, without storming a bus whose fault persists.
// (Ported from canH723.cpp.)
static void check_bus_off_once(uint8_t idx) {
    static bool was_bus_off[2] = { false, false };
    static uint32_t last_recover_ms[2] = { 0, 0 };
    if (!gPorts[idx].initialized || !gPorts[idx].drv) return;
    const bool bus_off = (gPorts[idx].drv->statusFlags() & (1U << 4)) != 0;
    if (bus_off != was_bus_off[idx]) {
        if (Serial) {
            Serial.print(bus_off ? "CANFD BUS-OFF on PORT" : "CANFD recovered on PORT");
            Serial.println(idx + 1);
        }
        was_bus_off[idx] = bus_off;
    }
    if (bus_off && (millis() - last_recover_ms[idx]) >= 1000U) {
        gPorts[idx].drv->recoverFromBusOff();
        last_recover_ms[idx] = millis();
    }
}

uint8_t CANMsgAvail(uint8_t port) {
    if (port == CAN_PORT_BOTH) {
        check_bus_off_once(0);
        check_bus_off_once(1);
        uint8_t n = 0;
        if (gPorts[0].initialized && gPorts[0].drv) n += gPorts[0].drv->driverReceiveFIFO0Count();
        if (gPorts[1].initialized && gPorts[1].drv) n += gPorts[1].drv->driverReceiveFIFO0Count();
        return n;
    }
    if (port > 1 || !gPorts[port].initialized || !gPorts[port].drv) return 0;
    check_bus_off_once(port);
    return gPorts[port].drv->driverReceiveFIFO0Count();
}

#endif // CANH7
