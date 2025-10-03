//
// Created by xie on 2025/9/27.
//

#include "dshot.h"

static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static float erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE / (MOTOR_POLE_COUNT / 2.0f);
bool useDshotTelemetry;

/// @brief To prepare the dshot packet
/// @param value
/// The throttle value ranges from 0 to 2047, with values from 48 to 2047 representing throttle levels from 0% to 100%.
/// Values from 0 to 47 are reserved for special commands.
/// @param requestTelemetry Wether to enable telemetry
static uint16_t dshot_prepare_packet(uint16_t value, bool requestTelemetry) {
    // throttle is 11 bits, so shift left 1 bit and add telemetry request bit to make 12 bits
    uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
    // checksum is 4 bits, so we need to shift left 4 bits and add it to make 16 bits
    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;  // xor data by nibbles
        csum_data >>= 4;
    }
#ifdef USE_TEMLEMETRY
    if (useDshotTelemetry) {
        csum = ~csum;  // BDshot telemetry request inverts the checksum
    }
#endif
    // Protect lower 4 bits
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;
    return packet;
}

///@brief 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer(uint32_t *motor_dmabuffer, uint16_t value, bool requestTelemetry) {
    uint16_t packet;
    packet = dshot_prepare_packet(value, requestTelemetry);

    for (int i = 0; i < 16; i++) {
        motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    motor_dmabuffer[16] = 0;
    motor_dmabuffer[17] = 0;
}

/// @brief Prepare the dshot dma buffer for all motors
/// @param motor_value The motor value array
static void dshot_prepare_dmabuffer_all(uint16_t *motor_value, bool requestTelemetry) {
    dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value[0], requestTelemetry);
}

/// @brief Decode the eRPM telemetry value from the ESC
static uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value) {
    // eRPM range
    if (value == 0x0fff) {
        return 0;
    }

    // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
    if (!value) {
        return DSHOT_TELEMETRY_INVALID;
    }

    // Convert period to erpm * 100
    return (1000000 * 60 / 100 + value / 2) / value;
}

float erpmToRpm(uint32_t erpm) {
    // rpm = (erpm * ERPM_PER_LSB) / (motorConfig()->motorPoleCount / 2)
    return erpm * erpmToHz * SECONDS_PER_MINUTE;
}


/// @brief Start the dshot timer
void dshot_start_pwm() { HAL_TIM_PWM_Start(MOTOR_1_TIM, MOTOR1_TIM_CHANNEL); }

/// @brief Start the dshot dma
static void dshot_dma_start() {
    HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor1_dmabuffer,
                     (uint32_t) &MOTOR_1_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
}

/// @brief Enable the dshot dma request
static void dshot_enable_dma_request() { __HAL_TIM_ENABLE_DMA(MOTOR_1_TIM, TIM_DMA_CC1); }

/// @brief Dma transfer complete callback
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
    }
}

/// @brief Put the dma transfer complete callback function to the timer dma handle
static void dshot_put_tc_callback_function() {
    // TIM_DMA_ID_CCx depends on timer channel
    MOTOR_1_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
}

/// @brief Send all zero signal to unlock the ESC
void esc_unlock(void) {
    /// Send zero signal to initialize the ESC
    uint32_t start = HAL_GetTick();
    uint16_t my_motor_value[4] = {0, 0, 0, 0};
    while (HAL_GetTick() - start < 3000) {
        dshot_send(my_motor_value, false);
    }
}

/// @brief Save the config to the ESC
void esc_save_config() {
    uint16_t motor_value[4] = {12, 12, 12, 12};
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 2000) {  // Todo: find fitable time
        dshot_send(motor_value, true);
    }
}

/// @brief Change the motor rotation direction
void dshot_change_rotation(uint8_t motor_index) {
    uint16_t motor_value[4] = {0, 0, 0, 0};
    motor_value[motor_index] = 7;
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 2000) {  // Todo: find fitable time
        dshot_send(motor_value, true);
    }
    esc_save_config();
}

/// @brief dshot init
void dshot_init(void) {
    printf("Dshot init start\r\n");
    dshot_put_tc_callback_function();
    dshot_start_pwm();
    esc_unlock();
    printf("Init complete\r\n");
}

/// @brief Write the motor value to the motor
void dshot_write(uint16_t *motor_value, bool requestTelemetry) {
    dshot_prepare_dmabuffer_all(motor_value, requestTelemetry);
    dshot_dma_start();
    dshot_enable_dma_request();
}

/// @brief Send the motor value to the motor
void dshot_send(uint16_t *motor_value, bool requestTelemetry) {
    dshot_write(motor_value, requestTelemetry);
    HAL_Delay(1);
}

/// @brief Dshot test loop
void dshot_loop(void) {
    uint16_t my_motor_value[4] = {0, 0, 0, 0};
    uint16_t value = 0;
    for (int i = 0; i < 4; i++) {
        my_motor_value[i] = value;
    }
    dshot_send(my_motor_value, false);
}
