#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Motor System States
 */
typedef enum
{
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ALIGNING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
    MOTOR_STATE_RESET
} MotorState_t;

/**
 * @brief Motor Directions
 */
typedef enum
{
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD
} MotorDirection_t;

// Configurable constants in header files
#define DEFAULT_SPEED_RAMP_RATE     500.0f  // RPM per second
#define ALIGNMENT_TIME_MS           200     // 200ms alignment phase
#define ALIGNMENT_DUTY_CYCLE        15      // 15% duty cycle
#define STALL_THRESHOLD_RPM         10      // Stall if RPM < 10
#define STALL_DUTY_THRESHOLD        70      // Stall if duty > 70%
#define STALL_TIME_LIMIT_MS         300     // Stall time limit in ms
#define CONTROL_LOOP_PERIOD_S       0.02f   // 20ms control loop period

// Prototypes
void Motor_Init(void);
void Motor_Process(void);
void Motor_ControlLoop_20ms(void);

void Motor_Enable(void);
void Motor_Disable(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetDirection(MotorDirection_t dir);
void Motor_SetSpeed(uint16_t rpm);
void Motor_ResetFault(void);
void Motor_Commutate(uint8_t hall_state);

MotorState_t Motor_GetState(void);
MotorDirection_t Motor_GetDirection(void);
uint16_t Motor_GetTargetSpeed(void);
uint16_t Motor_GetActualSpeed(void);
uint8_t Motor_GetPWMDuty(void);

// PID parameter configuration functions
void Motor_SetPID(float kp, float ki, float kd);
void Motor_GetPID(float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
