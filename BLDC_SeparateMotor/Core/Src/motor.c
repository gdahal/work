#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

#define TIM_PERIOD 8500  // 20kHz at 170MHz
#define TEST_DUTY_CYCLE 1700 // 20% duty cycle for initial testing

static MotorState_t current_motor_state = MOTOR_STATE_STOP;

// Phase States in Commutation: 0=Float, 1=Active(PWM), 2=Low(GND)
// Column 0: Phase A (TIM1 CH1)
// Column 1: Phase B (TIM1 CH2)
// Column 2: Phase C (TIM8 CH1)
static const uint8_t commutate_table_fwd[6][3] = {
    { 1, 2, 0 }, // State 1 (001): A+, B-
    { 1, 0, 2 }, // State 5 (101): A+, C-
    { 0, 1, 2 }, // State 4 (100): B+, C-
    { 2, 1, 0 }, // State 6 (110): B+, A-
    { 2, 0, 1 }, // State 2 (010): C+, A-
    { 0, 2, 1 }  // State 3 (011): C+, B-
};

static const uint8_t commutate_table_rev[6][3] = {
    { 2, 1, 0 }, // State 1 (001): A-, B+
    { 2, 0, 1 }, // State 5 (101): A-, C+
    { 0, 2, 1 }, // State 4 (100): B-, C+
    { 1, 2, 0 }, // State 6 (110): B+, A-
    { 1, 0, 2 }, // State 2 (010): C+, A-
    { 0, 1, 2 }  // State 3 (011): C+, B-
};

void Motor_Init(void)
{
    current_motor_state = MOTOR_STATE_STOP;
    Motor_Stop();
    
    // Enable DRV8302 (PA11 is EN_GATE)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

void Motor_StartForward(void)
{
    if (current_motor_state == MOTOR_STATE_FAULT) return;
    current_motor_state = MOTOR_STATE_FORWARD;
    
    // Enable main outputs
    htim1.Instance->BDTR |= TIM_BDTR_MOE;
    htim8.Instance->BDTR |= TIM_BDTR_MOE;
    
    // Start PWM timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    
    // Commutate immediately based on current Hall state
    uint8_t hall_state = Hall_GetState();
    Motor_Commutate(hall_state);
}

void Motor_StartBackward(void)
{
    if (current_motor_state == MOTOR_STATE_FAULT) return;
    current_motor_state = MOTOR_STATE_BACKWARD;
    
    // Enable main outputs
    htim1.Instance->BDTR |= TIM_BDTR_MOE;
    htim8.Instance->BDTR |= TIM_BDTR_MOE;
    
    // Start PWM timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    
    // Commutate immediately based on current Hall state
    uint8_t hall_state = Hall_GetState();
    Motor_Commutate(hall_state);
}

void Motor_Stop(void)
{
    if (current_motor_state != MOTOR_STATE_FAULT)
    {
        current_motor_state = MOTOR_STATE_STOP;
    }
    
    // Set all duty cycles to 0
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim8.Instance->CCR1 = 0;
    
    // Float all phases by disabling outputs
    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    htim8.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    
    // Disable Main Output Enable (MOE)
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    htim8.Instance->BDTR &= ~TIM_BDTR_MOE;
    
    // Stop PWM timers
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

MotorState_t Motor_GetState(void)
{
    return current_motor_state;
}

void Motor_Process(void)
{
    // Safety check: nFAULT (PA0) is active-low
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        Motor_Stop();
        current_motor_state = MOTOR_STATE_FAULT;
    }
}

void Motor_Commutate(uint8_t hall_state)
{
    // Safe shutdown on invalid state (000 or 111) or if motor is stopped
    if (hall_state == 0 || hall_state == 7 || 
        current_motor_state == MOTOR_STATE_STOP || 
        current_motor_state == MOTOR_STATE_FAULT)
    {
        Motor_Stop();
        return;
    }
    
    uint32_t pulse = TEST_DUTY_CYCLE;
    
    uint32_t ccer1 = htim1.Instance->CCER;
    uint32_t ccer8 = htim8.Instance->CCER;
    
    // Clear Enable and Complementary Enable bits for Phase A, Phase B, Phase C
    ccer1 &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    ccer8 &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    
    uint32_t ccr1_1 = 0; // Phase A
    uint32_t ccr1_2 = 0; // Phase B
    uint32_t ccr8_1 = 0; // Phase C
    
    // Select correct commutation table row
    // Hall states 1 to 6 correspond to indices 0 to 5 in our logic.
    // Hall sequence: H1, H2, H3 (PB6, PB7, PB8)
    uint8_t table_idx = 0;
    switch (hall_state)
    {
        case 1: table_idx = 0; break; // 001
        case 5: table_idx = 1; break; // 101
        case 4: table_idx = 2; break; // 100
        case 6: table_idx = 3; break; // 110
        case 2: table_idx = 4; break; // 010
        case 3: table_idx = 5; break; // 011
        default: Motor_Stop(); return; // Extra check
    }
    
    const uint8_t *table = (current_motor_state == MOTOR_STATE_FORWARD) ? 
                            commutate_table_fwd[table_idx] : commutate_table_rev[table_idx];
                            
    // Phase A (TIM1 CH1)
    if (table[0] == 1)      // Active PWM
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
    
    // Update comparison registers and enable flags
    htim1.Instance->CCR1 = ccr1_1;
    htim1.Instance->CCR2 = ccr1_2;
    htim8.Instance->CCR1 = ccr8_1;
    
    htim1.Instance->CCER = ccer1;
    htim8.Instance->CCER = ccer8;
}
