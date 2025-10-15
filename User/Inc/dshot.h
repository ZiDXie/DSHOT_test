//
// Created by xie on 2025/9/22.
//

#ifndef RM_DSHOT_DSHOT_H
#define RM_DSHOT_DSHOT_H

// User include
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "tim.h"

#define USE_TEMLEMETRY

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define DSHOT600_HZ MHZ_TO_HZ(12)
#define DSHOT300_HZ MHZ_TO_HZ(6)

#define DSHOT_TELEMETRY_NOEDGE (0xfffe)
#define DSHOT_TELEMETRY_INVALID (0xffff)
#define ERPM_PER_LSB 100.0f
#define SECONDS_PER_MINUTE 60.0f
#define MOTOR_POLE_COUNT 14.0  // Should be set according to the actual motor pole count
#define MOTOR_KV 24.48f  // Should be set according to the actual motor KV value
#define MOTOR_MAX_VOLTAGE 24.0f  // Should be set according to the actual battery voltage
#define MOTOR_MAX_RPM (MOTOR_KV * MOTOR_MAX_VOLTAGE)

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

/// Normally, a DSHOT frame contains only 16 bits of data,
/// but since the TIM BURST DMA method requires resetting at the end of the frame to prevent continuous signal output,
/// two additional bits are added at the end with actual compare values of 0.
#define DSHOT_DMA_BUFFER_SIZE 18
#define BIDSHOT_RESPONSE_BUFFER_SIZE 22
#define MIN_GCR_EDGES (7)
#define DSHOT_TELEMETRY_DEADTIME_US (30 + 5)  // 30 to switch lines and 5 to switch lines back

// test
#define MOTOR_1_TIM (&htim1)
#define MOTOR1_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR1_PIN_GPIO_PORT GPIOA
#define MOTOR1_PIN GPIO_PIN_8
#define MOTOR_2_TIM (&htim1)
#define MOTOR2_TIM_CHANNEL TIM_CHANNEL_2
#define MOTOR2_PIN_GPIO_PORT GPIOA
#define MOTOR2_PIN GPIO_PIN_9

void dshot_init(void);
void dshot_write(uint16_t* motor_value, bool requestTelemetry);
void dshot_send(uint16_t* motor_value, bool requestTelemetry);
void dshot_loop(void);
void dshot_set_output(uint8_t motor_index);
static void dshot_dma_tc_callback(DMA_HandleTypeDef* hdma);
#endif  // RM_DSHOT_DSHOT_H
