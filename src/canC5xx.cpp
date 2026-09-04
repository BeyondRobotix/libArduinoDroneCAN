#ifdef CANC532
#include "Arduino.h"
#include "canC5xx.h"
#include <string.h>

// See the UNVERIFIED note in canC5xx.h before relying on this driver.

static hal_fdcan_handle_t hfdcan1;
static bool g_initialized = false;

static uint32_t bitrate_to_hz(BITRATE bitrate)
{
    switch (bitrate) {
        case CAN_50KBPS:   return 50000;
        case CAN_100KBPS:  return 100000;
        case CAN_125KBPS:  return 125000;
        case CAN_250KBPS:  return 250000;
        case CAN_500KBPS:  return 500000;
        case CAN_1000KBPS: return 1000000;
    }
    return 1000000;
}

// Classic-CAN nominal bit timing: fixed 18 time-quanta/bit (1 sync + 13 seg1
// + 4 seg2, ~78% sample point) and a prescaler sized off the FDCAN kernel
// clock to land on the requested bitrate. Returns false if the kernel clock
// doesn't divide evenly (bit timing still gets programmed with the rounded
// prescaler -- an inexact fit shows up as bus errors, not an init failure).
static bool compute_nominal_timing(uint32_t kernel_clk_hz, uint32_t bitrate_hz,
                                    hal_fdcan_nominal_bit_timing_t *timing)
{
    static constexpr uint32_t kTqPerBit = 18;
    timing->nominal_jump_width = 4;
    timing->nominal_time_seg1  = 13;
    timing->nominal_time_seg2  = 4;

    if (kernel_clk_hz == 0 || bitrate_hz == 0) {
        timing->nominal_prescaler = 1;
        return false;
    }
    const uint32_t total_tq_hz = bitrate_hz * kTqPerBit;
    uint32_t prescaler = kernel_clk_hz / total_tq_hz;
    if (prescaler < 1) prescaler = 1;
    if (prescaler > 512) prescaler = 512;
    timing->nominal_prescaler = prescaler;
    return (kernel_clk_hz == prescaler * total_tq_hz);
}

bool CANInit(BITRATE bitrate, uint8_t port)
{
    if (port != CAN_PORT1) {
        return false;
    }
    if (g_initialized) {
        return true;
    }

    // --- GPIO: PA11 = FDCAN1_RX, PA12 = FDCAN1_TX, AF9 -------------------
    // (see br_platformio_hwdef's variants/MicroNodeC5/PeripheralPins.c)
    HAL_RCC_GPIOA_EnableClock();
    hal_gpio_config_t gpio_config = {};
    gpio_config.mode        = HAL_GPIO_MODE_ALTERNATE;
    gpio_config.pull        = HAL_GPIO_PULL_NO;
    gpio_config.speed       = HAL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
    gpio_config.alternate   = HAL_GPIO_AF_9;
    if (HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_11 | HAL_GPIO_PIN_12, &gpio_config) != HAL_OK) {
        return false;
    }

    // --- FDCAN1 kernel clock: PCLK1. No HSE crystal is assumed (no real ---
    // schematic exists for this board yet), so PCLK1 (derived from SYSCLK,
    // itself HSIS) is the only source that needs no extra clock wiring.
    HAL_RCC_FDCAN_EnableClock();
    if (HAL_RCC_FDCAN_SetKernelClkSource(HAL_RCC_FDCAN_CLK_SRC_PCLK1) != HAL_OK) {
        return false;
    }
    const uint32_t kernel_clk_hz = HAL_RCC_FDCAN_GetKernelClkFreq();

    hfdcan1 = (hal_fdcan_handle_t){};
    if (HAL_FDCAN_Init(&hfdcan1, HAL_FDCAN1) != HAL_OK) {
        return false;
    }

    hal_fdcan_config_t config = {};
    compute_nominal_timing(kernel_clk_hz, bitrate_to_hz(bitrate), &config.nominal_bit_timing);
    // Data bit timing is unused in classic mode, but HAL_FDCAN_SetConfig
    // still range-checks it, so it must hold register-legal values.
    config.data_bit_timing.data_prescaler  = 1;
    config.data_bit_timing.data_jump_width = 1;
    config.data_bit_timing.data_time_seg1  = 2;
    config.data_bit_timing.data_time_seg2  = 1;
    config.mode               = HAL_FDCAN_MODE_NORMAL;
    config.frame_format        = HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN;
    config.auto_retransmission = HAL_FDCAN_AUTO_RETRANSMISSION_ENABLE;
    config.transmit_pause      = HAL_FDCAN_TRANSMIT_PAUSE_DISABLE;
    config.protocol_exception  = HAL_FDCAN_PROTOCOL_EXCEPTION_ENABLE;
    config.std_filters_nbr     = 0;
    config.ext_filters_nbr     = 0;
    config.tx_fifo_queue_mode  = HAL_FDCAN_TX_MODE_FIFO;

    if (HAL_FDCAN_SetConfig(&hfdcan1, &config) != HAL_OK) {
        return false;
    }

    // No filter elements configured (std/ext_filters_nbr = 0) -- DroneCAN
    // filters at the libcanard layer, not in hardware, so route every
    // non-matching frame into Rx FIFO0 and drop remote frames (DroneCAN
    // never uses RTR).
    hal_fdcan_global_filter_config_t global_filter = {};
    global_filter.acceptance_non_matching_std = HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0;
    global_filter.acceptance_non_matching_ext = HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0;
    global_filter.acceptance_remote_std       = HAL_FDCAN_REMOTE_REJECT;
    global_filter.acceptance_remote_ext       = HAL_FDCAN_REMOTE_REJECT;
    if (HAL_FDCAN_SetGlobalFilter(&hfdcan1, &global_filter) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }

    g_initialized = true;
    return true;
}

bool CANSend(const CanardCANFrame *tx_msg, uint8_t port)
{
    if (port != CAN_PORT1 || !g_initialized || !tx_msg) {
        return false;
    }

    hal_fdcan_tx_header_t tx_header = {};
    tx_header.b.identifier      = tx_msg->id & 0x1FFFFFFFU;
    tx_header.b.frame_type      = HAL_FDCAN_FRAME_DATA;
    tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED; // DroneCAN always uses 29-bit IDs
    tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
    tx_header.b.data_length     = (hal_fdcan_data_length_code_t)(tx_msg->data_len & 0xFU);
    tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
    tx_header.b.frame_format    = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
    tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
    tx_header.b.message_marker  = 0;

    // ReqTransmitMsgFromFIFOQ returns HAL_ERROR (among other things) when the
    // Tx FIFO is full -- treat any non-OK result as "not accepted" so the
    // caller retries next cycle rather than dropping the frame.
    return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(&hfdcan1, &tx_header, tx_msg->data) == HAL_OK;
}

// Edge-triggered bus-off detector + 1 Hz recovery retry, mirroring canH743.cpp.
static void check_bus_off_once()
{
    static bool was_bus_off = false;
    static uint32_t last_recover_ms = 0;
    if (!g_initialized) return;

    hal_fdcan_protocol_status_t status = {};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK) return;
    const bool bus_off = (status.bus_off == HAL_FDCAN_BUS_OFF_FLAG_SET);
    if (bus_off != was_bus_off) {
        if (Serial) {
            Serial.println(bus_off ? "FDCAN BUS-OFF" : "FDCAN recovered");
        }
        was_bus_off = bus_off;
    }
    if (bus_off && (millis() - last_recover_ms) >= 1000U) {
        HAL_FDCAN_Recover(&hfdcan1);
        last_recover_ms = millis();
    }
}

void CANReceive(CanardCANFrame *rx_msg, uint8_t port)
{
    if (!rx_msg) return;
    rx_msg->id = 0;
    if (port != CAN_PORT1 || !g_initialized) return;

    uint32_t fill_level = 0;
    HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, HAL_FDCAN_RX_FIFO_0, &fill_level);
    if (fill_level == 0) return;

    hal_fdcan_rx_header_t rx_header = {};
    if (HAL_FDCAN_GetReceivedMessage(&hfdcan1, HAL_FDCAN_RX_FIFO_0, &rx_header, rx_msg->data) != HAL_OK) {
        return;
    }
    if (rx_header.b.identifier_type != HAL_FDCAN_ID_EXTENDED) {
        // DroneCAN only ever uses extended IDs -- drop anything else.
        rx_msg->data_len = 0;
        return;
    }
    rx_msg->id       = (rx_header.b.identifier & 0x1FFFFFFFU) | CANARD_CAN_FRAME_EFF;
    rx_msg->data_len = rx_header.b.data_length; // 0-8 for classic frames: code == byte count
}

uint8_t CANMsgAvail(uint8_t port)
{
    if (port != CAN_PORT1 || !g_initialized) return 0;
    check_bus_off_once();
    uint32_t fill_level = 0;
    HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, HAL_FDCAN_RX_FIFO_0, &fill_level);
    return (uint8_t)fill_level;
}

#endif // CANC532
