//
// Created by xie on 2025/9/27.
//

#include "dshot.h"

static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
bool TIMER_OUTPUT_INVERTED = false;
GPIO_TypeDef *PORT[2] = {MOTOR1_PIN_GPIO_PORT, MOTOR2_PIN_GPIO_PORT};
uint16_t PIN[2] = {MOTOR1_PIN, MOTOR2_PIN};
uint32_t TIM_CH[2] = {TIM_CHANNEL_1, TIM_CHANNEL_2};
uint32_t TIM_DMA_CC[2] = {TIM_DMA_CC1, TIM_DMA_CC2};
uint32_t TIM_DMA_ID[2] = {TIM_DMA_ID_CC1, TIM_DMA_ID_CC2};
bool is_input[2] = {false, false};

#ifdef USE_TEMLEMETRY
static uint32_t motor_response_buffer[2][BIDSHOT_RESPONSE_BUFFER_SIZE];
static float erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE / (MOTOR_POLE_COUNT / 2.0f);
float rpm[2] = {0.0f, 0.0f};
bool is_input[2] = {false, false};
bool useDshotTelemetry = false;
int32_t dshotTelemetryDeadtimeUs;
uint32_t inputStampUs;
#endif

/// @brief To prepare the dshot packet
/// @param value
/// The throttle value ranges from 0 to 2047, with values from 48 to 2047 representing throttle levels from 0%
/// to 100%. Values from 0 to 47 are reserved for special commands.
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

/// 16 bits packet to 16 pwm signal
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

/// Convert rpm to dshot value
uint16_t rpm_to_dshot_value(float rpm) {
    if (rpm > MOTOR_MAX_RPM) {
        rpm = MOTOR_MAX_RPM;
    } else if (rpm < 0) {
        rpm = 0;
    }
    float percent = rpm / MOTOR_MAX_RPM;
    int dshot_value = (int) (48 + (percent * (2047 - 48)));
    // Clamp the value to the valid range
    if (dshot_value < 48) {
        dshot_value = 48;
    } else if (dshot_value > 2047) {
        dshot_value = 2047;
    }
    return (uint16_t) dshot_value;
}

#ifdef USE_TEMLEMETRY
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void) { return HAL_GetTick() * 1000 + (uint32_t) (DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000)); }

/// Decode the eRPM telemetry value from the ESC
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

float erpmToRpm(uint32_t erpm) { return erpm * erpmToHz * SECONDS_PER_MINUTE; }

/// Get rpm from the telemetry value
static uint32_t decode_telemetry_packet(const uint32_t buffer[], uint32_t count) {
    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;
            }
            len = (diff + 8) / 16;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
                                        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8,  1,  0, 4,  12, 0};

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8);  // xor bytes
    csum = csum ^ (csum >> 4);  // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return DSHOT_TELEMETRY_INVALID;
    }

    return decodedValue >> 4;
}

bool dshot_temelemetry_decode() {
    if (!useDshotTelemetry) {
        return false;
    }
    const uint32_t currentUs = micros();
    int32_t usSinceInput = currentUs - inputStampUs;
    if (usSinceInput >= 0 && usSinceInput < DSHOT_TELEMETRY_DEADTIME_US) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        if (is_input[i]) {
            uint32_t edges = BIDSHOT_RESPONSE_BUFFER_SIZE - __HAL_DMA_GET_COUNTER();
            __HAL_TIM_DISABLE_DMA();
            uint16_t rawValue;
            if (edges > MIN_GCR_EDGES) {
                rawValue = decode_telemetry_packet(, edges);
                if (rawValue != DSHOT_TELEMETRY_INVALID) {
                    uint32_t erpm = dshot_decode_eRPM_telemetry_value(rawValue);
                    if (erpm != DSHOT_TELEMETRY_INVALID) {
                        rpm[i] = erpmToRpm(erpm);
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
        dshot_set_output(i);
    }
    inputStampUs = 0;
    return true;
}

/// DMA transfer complete callback for input capture mode
static void dshot_ic_dma_tc_callback(DMA_HandleTypeDef *hdma) {}

/// Set the pin and timer channel to input capture mode
void dshot_set_input(uint8_t motor_index) {
    // Set the pin to input mode with pull-up resistor
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PIN[motor_index];
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PORT[motor_index], &GPIO_InitStruct);

    is_input[motor_index] = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }

    // Set the timer channel to input capture mode
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.Period = 0xffffffff;  // if 16 bit timer, set to 0xffff,32 bit timer, set to 0xffffffff
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_IC_Init(&htim1);

    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 2;
    HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CH[motor_index]);

    // Reinitialize the DMA associated with the timer channel
    DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID[motor_index]];
    if (hdma) {
        HAL_DMA_Abort(hdma);
        HAL_DMA_DeInit(hdma);
        hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma->Init.PeriphInc = DMA_PINC_DISABLE;
        hdma->Init.MemInc = DMA_MINC_ENABLE;
        hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma->Init.Mode = DMA_NORMAL;
        hdma->Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(hdma);
        hdma->XferCpltCallback = dshot_ic_dma_tc_callback;
    }
}
#endif

/// Set the pin and timer channel to output pwm mode
void dshot_set_output(uint8_t motor_index) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PIN[motor_index];
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PORT[motor_index], &GPIO_InitStruct);

    is_input[motor_index] = false;

    TIM_OC_InitTypeDef sConfigOC = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 12 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim1);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIMER_OUTPUT_INVERTED ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIMER_OUTPUT_INVERTED ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CH[motor_index]);

    DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID[motor_index]];
    if (hdma) {
        HAL_DMA_Abort(hdma);
        HAL_DMA_DeInit(hdma);
        hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma->Init.PeriphInc = DMA_PINC_DISABLE;
        hdma->Init.MemInc = DMA_MINC_ENABLE;
        hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma->Init.Mode = DMA_NORMAL;
        hdma->Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(hdma);
        hdma->XferCpltCallback = dshot_dma_tc_callback;
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CH[motor_index]);
}

/// @brief Prepare the dshot dma buffer for all motors
/// @param motor_value The motor value array
static void dshot_prepare_dmabuffer_all(uint16_t *motor_value, bool requestTelemetry) {
    dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value[0], requestTelemetry);
    dshot_prepare_dmabuffer(motor2_dmabuffer, motor_value[1], requestTelemetry);
}

/// Start the dshot dma
static void dshot_dma_start() {
    HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor1_dmabuffer,
                     (uint32_t) &MOTOR_1_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_2_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t) motor2_dmabuffer,
                     (uint32_t) &MOTOR_2_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
}

/// Enable the dshot dma request
static void dshot_enable_dma_request() {
    __HAL_TIM_ENABLE_DMA(MOTOR_1_TIM, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(MOTOR_2_TIM, TIM_DMA_CC2);
}

/// Dma transfer complete callback
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;
    int motor_index = -1;
    if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
        motor_index = 0;
    }
    if (hdma == htim->hdma[TIM_DMA_ID_CC2]) {
        motor_index = 1;
    }
    if (motor_index >= 0) {
        HAL_DMA_Abort(hdma);
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC[motor_index]);
#ifdef USE_TEMLEMETRY
        if (useDshotTelemetry) {
            dshot_set_input(motor_index);
            HAL_TIM_IC_Start_DMA(&htim1, TIM_CH[motor_index], (uint32_t *) motor_response_buffer[motor_index],
                                 BIDSHOT_RESPONSE_BUFFER_SIZE);
        }

#endif
    }
}

/// Send all zero signal to unlock the ESC
void esc_unlock(void) {
    /// Send zero signal to initialize the ESC
    uint32_t start = HAL_GetTick();
    uint16_t motor_value[4] = {0, 0, 0, 0};
    while (HAL_GetTick() - start < 3000) {
        dshot_send(motor_value, false);
    }
}

/// @brief change the motor rotation direction
/// @param clockwise
void motor_change_rotation(uint16_t motor_index, bool clockwise) {
    uint16_t command = 0;
    if (clockwise) {
        command = 8;
    } else {
        command = 7;
    }
    uint32_t start = HAL_GetTick();
    uint16_t motor_value[4] = {0, 0, 0, 0};
    motor_value[motor_index] = command;
    while (HAL_GetTick() - start < 50) {
        dshot_send(motor_value, true);
    }
}

/// configure the motor
void motor_configure() {
    motor_change_rotation(0, true);
    motor_change_rotation(1, false);
#ifdef USE_TEMLEMETRY
    useDshotTelemetry = true;
    TIMER_OUTPUT_INVERTED = true;
    dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 * (16 * MOTOR_BITLENGTH) / DSHOT300_HZ;
#endif
}

/// dshot init
void dshot_init(void) {
    printf("Dshot init start\r\n");
    dshot_set_output(0);
    dshot_set_output(1);
    esc_unlock();
    motor_configure();
    printf("Init complete\r\n");
}

/// Write the motor value to the motor
void dshot_write(uint16_t *motor_value, bool requestTelemetry) {
    dshot_prepare_dmabuffer_all(motor_value, requestTelemetry);
    dshot_dma_start();
    dshot_enable_dma_request();
}

/// Send the motor value to the motor
void dshot_send(uint16_t *motor_value, bool requestTelemetry) {
    dshot_write(motor_value, requestTelemetry);
    HAL_Delay(1);
}

/// Dshot test loop
void dshot_loop(void) {
    uint16_t motor_value[4] = {0, 0, 0, 0};
    uint16_t command = 100;
    for (int i = 0; i < 4; i++) {
        motor_value[i] = command;
    }
    dshot_send(motor_value, false);
}
