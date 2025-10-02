//
// Created by xie on 2025/9/22.
//

#ifndef RM_DSHOT_DSHOT_H
#define RM_DSHOT_DSHOT_H

// User include
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "tim.h"

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define MOTOR_DSHOT600_HZ MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ MHZ_TO_HZ(3)

typedef enum {
    MOTOR_PROTOCOL_DSHOT150,
    MOTOR_PROTOCOL_DSHOT300,
    MOTOR_PROTOCOL_DSHOT600,
} motorProtocolTypes_e;

/// For dshot600 on 168MHz F407
/// The term of pwm is 1.67us,the bit1 is 1.25us high level, and the bit0 is 0.625us high level
#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

/// Normally, a DSHOT frame contains only 16 bits of data,
/// but since the TIM BURST DMA method requires resetting at the end of the frame to prevent continuous signal output,
/// two additional bits are added at the end with actual compare values of 0.
#define DSHOT_DMA_BUFFER_SIZE 18

// test
#define MOTOR_1_TIM (&htim1)
#define MOTOR1_TIM_CHANNEL TIM_CHANNEL_1

void dshot_init(void);
void dshot_write(uint16_t* motor_value, bool requestTelemetry);
void dshot_send(uint16_t* motor_value, bool requestTelemetry);

#endif  // RM_DSHOT_DSHOT_H
