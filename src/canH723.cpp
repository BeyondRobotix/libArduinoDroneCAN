#ifdef ARDUINO_NUCLEO_H723ZG
#include "Arduino.h"
#include "canH723.h"

// --- Message RAM Configuration ---
// These values MUST be defined as macros before including the ACANFD_STM32.h header.
// The library's internal headers use these macros to configure the FDCAN peripherals.
#define FDCAN1_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN2_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN3_MESSAGE_RAM_WORD_SIZE 800

// The ACANFD_STM32 library requires this main header to be included in one .cpp file
// to instantiate the CAN objects (fdcan1, fdcan2, fdcan3).
#include <ACANFD_STM32.h>

// --- FDCAN peripheral handles ---
// An array to easily select the CAN interface at runtime.
static ACANFD_STM32* can_ifaces[] = {&fdcan1, &fdcan2, &fdcan3};
// Pointer to the currently active CAN interface.
static ACANFD_STM32* active_can_iface = nullptr;
// Maximum number of supported CAN interfaces on the H723ZG.
const int MAX_CAN_IFACES = sizeof(can_ifaces) / sizeof(can_ifaces[0]);


static bool select_iface(int can_iface_index) {
    // Driver currently pins to FDCAN1 regardless of the requested index.
    can_iface_index = 0;
    if (can_iface_index < 0 || can_iface_index >= MAX_CAN_IFACES) {
        return false;
    }
    active_can_iface = can_ifaces[can_iface_index];
    return true;
}

static uint32_t bitrate_to_hz(BITRATE bitrate) {
    switch (bitrate) {
        case CAN_50KBPS:   return  50 * 1000;
        case CAN_100KBPS:  return 100 * 1000;
        case CAN_125KBPS:  return 125 * 1000;
        case CAN_250KBPS:  return 250 * 1000;
        case CAN_500KBPS:  return 500 * 1000;
        case CAN_1000KBPS: return 1000 * 1000;
    }
    return 1000 * 1000;
}

/**
 * @brief Initializes the FDCAN controller in classic CAN mode (no FD, x1 data rate).
 */
bool CANInit(BITRATE bitrate, int can_iface_index) {
    if (!select_iface(can_iface_index)) return false;

    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor::x1);
    settings.mTxPin = PD_1;
    settings.mRxPin = PD_0;

    return active_can_iface->beginFD(settings) == 0;
}

/**
 * @brief Initializes the FDCAN controller in CAN-FD mode (NORMAL_FD, 4x data rate).
 * Matches ArduPilot's typical 1M arb / 4M data configuration.
 */
bool CANInit_fd(BITRATE bitrate, int can_iface_index) {
    if (!select_iface(can_iface_index)) return false;

    ACANFD_STM32_Settings settings(bitrate_to_hz(bitrate), DataBitRateFactor::x4);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
    settings.mTxPin = PD_1;
    settings.mRxPin = PD_0;

    return active_can_iface->beginFD(settings) == 0;
}

/**
 * @brief Sends a CAN message using the initialized FDCAN peripheral.
 *
 * This function converts a CanardCANFrame to the library's CANFDMessage format.
 * It forces all outgoing messages to use the Extended ID format for Ardupilot compatibility.
 *
 * @param tx_msg A pointer to the CanardCANFrame to be sent.
 */
void CANSend(const CanardCANFrame *tx_msg) {
    if (!active_can_iface || !tx_msg) {
        return;
    }

    CANFDMessage message;
    // ArduPilot's CAN drivers exclusively use extended frames.
    message.ext = true;
    message.id = tx_msg->id & CANARD_CAN_EXT_ID_MASK;
    message.len = tx_msg->data_len;
    memcpy(message.data, tx_msg->data, tx_msg->data_len);

#if CANARD_ENABLE_CANFD
    message.type = tx_msg->canfd
                       ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                       : CANFDMessage::CAN_DATA;
#else
    message.type = CANFDMessage::CAN_DATA;
#endif

    uint32_t send_status = active_can_iface->tryToSendReturnStatusFD(message);
    if (send_status != 0) {
        if (Serial) Serial.println("Failed to send CAN message");
    }
}


/**
 * @brief Receives a CAN message if one is available.
 *
 * This function checks the RX FIFO 0, and if a message is present, it populates
 * the provided CanardCANFrame struct.
 *
 * @param rx_msg A pointer to a CanardCANFrame that will be filled with the received data.
 */
void CANReceive(CanardCANFrame *rx_msg) {
    if (!active_can_iface || !rx_msg) {
        return;
    }

    CANFDMessage message;
    // Check RX FIFO 0 for a new message.
    if (active_can_iface->receiveFD0(message)) {
        // Populate the Canard frame from the received library message.
        rx_msg->id = (message.id & 0x1FFFFFFFU) | CANARD_CAN_FRAME_EFF;
        rx_msg->data_len = message.len;
        memcpy(rx_msg->data, message.data, message.len);
#if CANARD_ENABLE_CANFD
        rx_msg->canfd = (message.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                      || message.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH);
#endif
    } else {
        // No message received, set ID to 0 to indicate an invalid/empty frame.
        rx_msg->id = 0;
        rx_msg->data_len = 0;
    }
}

/**
 * @brief Checks for available CAN messages.
 *
 * @return The number of messages pending in the driver's software receive FIFO 0.
 */
uint8_t CANMsgAvail(void) {
    if (!active_can_iface) {
        return 0;
    }
    // Return the number of messages available in RX FIFO 0.
    return active_can_iface->availableFD0();
}

#endif // ARDUINO_NUCLEO_H723ZG

