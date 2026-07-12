#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Motor States for safe operation
 */
typedef enum
{
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_BACKWARD,
    MOTOR_STATE_FAULT
} MotorState_t;

// Public function prototypes
void Motor_Init(void);
void Motor_StartForward(void);
void Motor_StartBackward(void);
void Motor_Stop(void);
void Motor_Commutate(uint8_t hall_state);
MotorState_t Motor_GetState(void);
void Motor_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
