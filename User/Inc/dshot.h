//
// Created by xie on 2025/9/22.
//

#ifndef RM_DSHOT_DSHOT_H
#define RM_DSHOT_DSHOT_H

// User include
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
#define DSHOT150_HZ MHZ_TO_HZ(3)
#define BIDSHOT_RESPONSE_HZ MHZ_TO_HZ(6 * 5 / 4)  // For dshot600

#define DSHOT_TELEMETRY_NOEDGE (0xfffe)
#define DSHOT_TELEMETRY_INVALID (0xffff)
#define ERPM_PER_LSB 100.0f
#define SECONDS_PER_MINUTE 60.0f
#define MOTOR_POLE_COUNT 14.0  // Should be set according to the actual motor pole count

typedef enum {
    MOTOR_PROTOCOL_DSHOT150,
    MOTOR_PROTOCOL_DSHOT300,
    MOTOR_PROTOCOL_DSHOT600,
} motorProtocolTypes_e;

/// For dshot600.The term of pwm is 1.67us,the bit1 is 1.25us high level, and the bit0 is 0.625us high level
#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

/// Normally, a DSHOT frame contains only 16 bits of data,
/// but since the TIM BURST DMA method requires resetting at the end of the frame to prevent continuous signal output,
/// two additional bits are added at the end with actual compare values of 0.
#define DSHOT_DMA_BUFFER_SIZE 18
#define BIDSHOT_RESPONSE_BUFFER_SIZE 21

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

#endif  // RM_DSHOT_DSHOT_H
