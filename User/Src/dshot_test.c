//
// Created by xie on 2025/9/27.
//

#include "dshot.h"

static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

/// @brief To check CRC and prepare the dshot packet
/// @param value
/// The throttle value ranges from 0 to 2047, with values from 48 to 2047 representing throttle levels from 0% to 100%.
/// Values from 0 to 47 are reserved for special commands.
/// @param requestTelemetry
/// Whether to request the return of data
/// @return Dshot packet
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
    csum = ~csum;  // BDshot telemetry request inverts the checksum
#endif
    // Protect lower 4 bits
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;
    return packet;
}

// Convert 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer(uint32_t *motor_dmabuffer, uint16_t value) {
    uint16_t packet;
    packet = dshot_prepare_packet(value, false);

    for (int i = 0; i < 16; i++) {
        motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    motor_dmabuffer[16] = 0;
    motor_dmabuffer[17] = 0;
}

/// @brief Prepare the dshot dma buffer for all motors
/// @param motor_value The motor value array
static void dshot_prepare_dmabuffer_all(uint16_t *motor_value) {
    dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value[0]);
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
    /// Send some zero signal to initialize the ESC
    uint32_t start = HAL_GetTick();
    uint16_t my_motor_value[4] = {0, 0, 0, 0};
    while (HAL_GetTick() - start < 3000) {
        dshot_write(my_motor_value);
        HAL_Delay(1);
    }
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
void dshot_write(uint16_t *motor_value) {
    dshot_prepare_dmabuffer_all(motor_value);
    dshot_dma_start();
    dshot_enable_dma_request();
}

void dshot_loop(void) {
    uint16_t my_motor_value[4] = {0, 0, 0, 0};
    uint16_t value = 0;
    for (int i = 0; i < 4; i++) {
        my_motor_value[i] = value;
    }
    dshot_write(my_motor_value);
    HAL_Delay(1);
}
