#include "motor.h"
#include "main.h"
#include "pid.h"
#include "speed.h"

/* Extern timer variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

#define TIM_PERIOD 8500  // 20kHz at 170MHz

/* Commutation Tables (Phase A, Phase B, Phase C)
 * 0 = Float, 1 = Active PWM (High-Side PWM + Low-Side Complementary), 2 = Low (GND)
 */
static const uint8_t commutate_table_fwd[6][3] = {
    { 1, 2, 0 }, // State 1: A+, B-
    { 1, 0, 2 }, // State 5: A+, C-
    { 0, 1, 2 }, // State 4: B+, C-
    { 2, 1, 0 }, // State 6: B+, A-
    { 2, 0, 1 }, // State 2: C+, A-
    { 0, 2, 1 }  // State 3: C+, B-
};

static const uint8_t commutate_table_rev[6][3] = {
    { 2, 1, 0 }, // State 1: A-, B+
    { 2, 0, 1 }, // State 5: A-, C+
    { 0, 2, 1 }, // State 4: B-, C+
    { 1, 2, 0 }, // State 6: B+, A-
    { 1, 0, 2 }, // State 2: C+, A-
    { 0, 1, 2 }  // State 3: C+, B-
};

/* Motor State Variables */
static MotorState_t motor_state = MOTOR_STATE_INIT;
static MotorDirection_t motor_direction = MOTOR_DIR_STOP;
static MotorDirection_t requested_direction = MOTOR_DIR_STOP;

static float target_speed_rpm = 0.0f;
static float current_ramped_speed_rpm = 0.0f;
static float speed_ramp_rate = DEFAULT_SPEED_RAMP_RATE; // RPM/sec
static uint8_t pwm_duty = 0; // 0 to 100%

/* Alignment state variables */
static uint32_t alignment_start_time = 0;

/* Stall detection variables */
static uint32_t stall_timer = 0;

/* PID Speed controller */
static PID_Controller speed_pid;
extern bool new_speed_data_available; // Defined in main.c
extern volatile uint32_t last_hall_transition_time; // Defined in main.c

/* External UART message sender */
extern void UART_SendMessage(const char *msg);

/**
 * FLOWCHART: Startup Alignment & Control Transition
 * 
 *     [START]
 *        │
 *        ▼
 *   Enable Gate Driver (PA11 High)
 *        │
 *        ▼
 *   Set PWM to 15% Duty Cycle
 *        │
 *        ▼
 *   Force Commutation State 1 (A+, B-)
 *        │
 *        ▼
 *   Wait for 200 ms (Rotor Alignment)
 *        │
 *        ▼
 *   Read Hall Sensors & Synchronize
 *        │
 *        ▼
 *   Enter Closed-Loop PID Running State
 */

void Motor_Init(void)
{
    motor_state = MOTOR_STATE_INIT;
    motor_direction = MOTOR_DIR_STOP;
    requested_direction = MOTOR_DIR_STOP;
    target_speed_rpm = 0.0f;
    current_ramped_speed_rpm = 0.0f;
    pwm_duty = 0;
    
    // Initialize PID speed controller (Kp, Ki, Kd, min_out, max_out)
    PID_Init(&speed_pid, 0.08f, 0.04f, 0.005f, 0.0f, 100.0f);
    
    // Initially disable gate driver (PA11 Low)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    
    // Stop timers/PWM
    Motor_Stop();
    
    motor_state = MOTOR_STATE_READY;
}

void Motor_Enable(void)
{
    if (motor_state == MOTOR_STATE_READY || motor_state == MOTOR_STATE_INIT)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        motor_state = MOTOR_STATE_IDLE;
        UART_SendMessage("MSG: Gate Driver Enabled (IDLE)\r\n");
    }
}

void Motor_Disable(void)
{
    Motor_Stop();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    motor_state = MOTOR_STATE_READY;
    UART_SendMessage("MSG: Gate Driver Disabled\r\n");
}

void Motor_Start(void)
{
    if (motor_state == MOTOR_STATE_IDLE)
    {
        if (requested_direction == MOTOR_DIR_STOP)
        {
            requested_direction = MOTOR_DIR_FORWARD; // Default to forward
        }
        motor_direction = requested_direction;
        current_ramped_speed_rpm = 0.0f;
        
        PID_Reset(&speed_pid);
        Speed_Reset();
        last_hall_transition_time = HAL_GetTick();
        
        motor_state = MOTOR_STATE_ALIGNING;
        alignment_start_time = HAL_GetTick();
        
        // Start PWM timers
        htim1.Instance->BDTR |= TIM_BDTR_MOE;
        htim8.Instance->BDTR |= TIM_BDTR_MOE;
        
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        
        // Force Align State 1 (A+, B-)
        pwm_duty = ALIGNMENT_DUTY_CYCLE;
        Motor_Commutate(1); // Force state 1
        
        UART_SendMessage("MSG: Alignment Started...\r\n");
    }
}

void Motor_Stop(void)
{
    // Clear duty cycles
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim8.Instance->CCR1 = 0;
    
    // Disable outputs
    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    htim8.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    
    // Disable Main Output Enable (MOE)
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    htim8.Instance->BDTR &= ~TIM_BDTR_MOE;
    
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    
    PID_Reset(&speed_pid);
    pwm_duty = 0;
    
    if (motor_state != MOTOR_STATE_FAULT)
    {
        motor_state = MOTOR_STATE_IDLE;
    }
}

void Motor_SetDirection(MotorDirection_t dir)
{
    requested_direction = dir;
}

void Motor_SetSpeed(uint16_t rpm)
{
    target_speed_rpm = (float)rpm;
}

void Motor_ResetFault(void)
{
    if (motor_state == MOTOR_STATE_FAULT)
    {
        PID_Reset(&speed_pid);
        Speed_Reset();
        current_ramped_speed_rpm = 0.0f;
        target_speed_rpm = 0.0f;
        motor_state = MOTOR_STATE_READY;
        UART_SendMessage("MSG: Fault Cleared. Motor Ready.\r\n");
    }
}

MotorState_t Motor_GetState(void)
{
    return motor_state;
}

MotorDirection_t Motor_GetDirection(void)
{
    return motor_direction;
}

uint16_t Motor_GetTargetSpeed(void)
{
    return (uint16_t)target_speed_rpm;
}

uint16_t Motor_GetActualSpeed(void)
{
    return Speed_GetRPM();
}

uint8_t Motor_GetPWMDuty(void)
{
    return pwm_duty;
}

void Motor_SetPID(float kp, float ki, float kd)
{
    speed_pid.Kp = kp;
    speed_pid.Ki = ki;
    speed_pid.Kd = kd;
}

void Motor_GetPID(float *kp, float *ki, float *kd)
{
    *kp = speed_pid.Kp;
    *ki = speed_pid.Ki;
    *kd = speed_pid.Kd;
}

void Motor_Process(void)
{
    // External Fault Monitor (nFAULT active low)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        Motor_Stop();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // Disable Gate Driver
        motor_state = MOTOR_STATE_FAULT;
        UART_SendMessage("FAULT: nFAULT Active!\r\n");
    }
    
    // nOCTW Monitor (active low)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        Motor_Stop();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        motor_state = MOTOR_STATE_FAULT;
        UART_SendMessage("FAULT: Overcurrent/Thermal Warning!\r\n");
    }
}

/**
 * @brief 20ms Control Loop for Speed Ramping, PID update, Soft Stop, and Stall Check
 */
void Motor_ControlLoop_20ms(void)
{
    if (motor_state == MOTOR_STATE_ALIGNING)
    {
        // Check if 200 ms alignment has finished
        if (HAL_GetTick() - alignment_start_time >= ALIGNMENT_TIME_MS)
        {
            uint8_t current_hall = Hall_GetState();
            if (current_hall == 0 || current_hall == 7)
            {
                Motor_Stop();
                motor_state = MOTOR_STATE_FAULT;
                UART_SendMessage("FAULT: Invalid Hall State on Alignment Sync\r\n");
                return;
            }
            
            // Align finished, start closed loop commutation
            motor_state = MOTOR_STATE_RUNNING;
            Motor_Commutate(current_hall);
            UART_SendMessage("MSG: Closed-Loop Running\r\n");
        }
    }
    else if (motor_state == MOTOR_STATE_RUNNING)
    {
        // Safety check: Hall sensor timeout
        if (HAL_GetTick() - last_hall_transition_time > 500)
        {
            Motor_Stop();
            motor_state = MOTOR_STATE_FAULT;
            UART_SendMessage("FAULT: Hall Sensor Timeout (No transitions for 500ms)!\r\n");
            return;
        }

        // 1. Safe Direction Reversal & Ramping Speed Target
        if (motor_direction != requested_direction)
        {
            // Ramp target down to zero first
            if (current_ramped_speed_rpm > 0.0f)
            {
                current_ramped_speed_rpm -= speed_ramp_rate * CONTROL_LOOP_PERIOD_S;
                if (current_ramped_speed_rpm < 0.0f) current_ramped_speed_rpm = 0.0f;
            }
            
            // Once the actual and ramped speeds are safe, reset PID and switch direction
            if (Speed_GetRPM() < 50 && current_ramped_speed_rpm == 0.0f)
            {
                PID_Reset(&speed_pid);
                motor_direction = requested_direction;
                UART_SendMessage("MSG: Direction Switched\r\n");
            }
        }
        else
        {
            // Normal speed ramping
            if (current_ramped_speed_rpm < target_speed_rpm)
            {
                current_ramped_speed_rpm += speed_ramp_rate * CONTROL_LOOP_PERIOD_S;
                if (current_ramped_speed_rpm > target_speed_rpm) current_ramped_speed_rpm = target_speed_rpm;
            }
            else if (current_ramped_speed_rpm > target_speed_rpm)
            {
                current_ramped_speed_rpm -= speed_ramp_rate * CONTROL_LOOP_PERIOD_S;
                if (current_ramped_speed_rpm < target_speed_rpm) current_ramped_speed_rpm = target_speed_rpm;
            }
        }

        // 2. Soft Stop Check
        if (requested_direction == MOTOR_DIR_STOP && current_ramped_speed_rpm == 0.0f && Speed_GetRPM() < 30)
        {
            Motor_Stop();
            UART_SendMessage("MSG: Motor Stopped Softly\r\n");
            return;
        }

        // 3. Run PID Speed Control
        // Do not update faster than fresh speed measurements are available (optional block)
        if (new_speed_data_available)
        {
            new_speed_data_available = false; // Clear data flag
            
            float measured_rpm = (float)Speed_GetRPM();
            float pid_out = PID_Update(&speed_pid, current_ramped_speed_rpm, measured_rpm, CONTROL_LOOP_PERIOD_S);
            pwm_duty = (uint8_t)pid_out;
            
            // Re-commutate with the updated duty cycle
            Motor_Commutate(Hall_GetState());
        }
        
        // 4. Stall Detection
        if (pwm_duty > STALL_DUTY_THRESHOLD && Speed_GetRPM() < STALL_THRESHOLD_RPM)
        {
            if (stall_timer == 0)
            {
                stall_timer = HAL_GetTick();
            }
            else if (HAL_GetTick() - stall_timer >= STALL_TIME_LIMIT_MS)
            {
                Motor_Stop();
                motor_state = MOTOR_STATE_FAULT;
                UART_SendMessage("FAULT: Motor Stall Detected!\r\n");
            }
        }
        else
        {
            stall_timer = 0;
        }
    }
}

void Motor_Commutate(uint8_t hall_state)
{
    if (hall_state == 0 || hall_state == 7)
    {
        // Safe shutdown on invalid state pattern
        Motor_Stop();
        motor_state = MOTOR_STATE_FAULT;
        UART_SendMessage("FAULT: Invalid Hall Pattern!\r\n");
        return;
    }
    
    if (motor_state != MOTOR_STATE_RUNNING && motor_state != MOTOR_STATE_ALIGNING)
    {
        return;
    }
    
    // Scale duty to Pulse counts (TIM_PERIOD = 8500 = 100%)
    uint32_t pulse = (uint32_t)(pwm_duty * TIM_PERIOD) / 100;
    
    uint32_t ccer1 = htim1.Instance->CCER;
    uint32_t ccer8 = htim8.Instance->CCER;
    
    // Clear Enable (CCxE) and Complementary Enable (CCxNE) bits
    ccer1 &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    ccer8 &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    
    uint32_t ccr1_1 = 0;
    uint32_t ccr1_2 = 0;
    uint32_t ccr8_1 = 0;
    
    uint8_t table_idx = 0;
    switch (hall_state)
    {
        case 1: table_idx = 0; break;
        case 5: table_idx = 1; break;
        case 4: table_idx = 2; break;
        case 6: table_idx = 3; break;
        case 2: table_idx = 4; break;
        case 3: table_idx = 5; break;
        default: Motor_Stop(); return;
    }
    
    const uint8_t *table = (motor_direction == MOTOR_DIR_FORWARD) ? 
                            commutate_table_fwd[table_idx] : commutate_table_rev[table_idx];
                            
    // Phase A (TIM1 CH1)
    if (table[0] == 1)      // Active PWM (Complementary Switching)
    {
        ccr1_1 = pulse;
        ccer1 |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    }
    else if (table[0] == 2) // Low (GND)
    {
        ccr1_1 = 0;
        ccer1 |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    }
    
    // Phase B (TIM1 CH2)
    if (table[1] == 1)      // Active PWM
    {
        ccr1_2 = pulse;
        ccer1 |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    }
    else if (table[1] == 2) // Low (GND)
    {
        ccr1_2 = 0;
        ccer1 |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    }
    
    // Phase C (TIM8 CH1)
    if (table[2] == 1)      // Active PWM
    {
        ccr8_1 = pulse;
        ccer8 |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    }
    else if (table[2] == 2) // Low (GND)
    {
        ccr8_1 = 0;
        ccer8 |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    }
    
    // Write registers
    htim1.Instance->CCR1 = ccr1_1;
    htim1.Instance->CCR2 = ccr1_2;
    htim8.Instance->CCR1 = ccr8_1;
    
    htim1.Instance->CCER = ccer1;
    htim8.Instance->CCER = ccer8;
}
