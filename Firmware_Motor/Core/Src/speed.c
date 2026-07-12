#include "speed.h"

static uint32_t last_ticks = 0;
static uint16_t rpm_history[FILTER_SAMPLES] = {0};
static uint8_t filter_index = 0;
static float current_rpm = 0.0f;

/**
 * @brief Initialize speed variables
 */
void Speed_Init(void)
{
    Speed_Reset();
}

/**
 * @brief Reset speed calculations and filters
 */
void Speed_Reset(void)
{
    last_ticks = 0;
    filter_index = 0;
    current_rpm = 0.0f;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        rpm_history[i] = 0;
    }
}

/**
 * @brief Process Hall sensor state transition to calculate period and RPM
 * @param current_ticks The microsecond timestamp (from TIM2 free running counter)
 */
void Speed_ProcessTransition(uint32_t current_ticks)
{
    if (last_ticks == 0) {
        last_ticks = current_ticks;
        return; // Need two transitions to measure period
    }
    
    uint32_t elapsed_ticks = current_ticks - last_ticks;
    last_ticks = current_ticks;
    
    if (elapsed_ticks == 0) {
        return; // Prevent division by zero
    }
    
    // Calculate RPM based on elapsed time between transitions
    // RPM = 60 / (period_for_one_mechanical_rev)
    // 1 electrical rev = HALL_EDGES_PER_ELEC_REV * transition_period
    // 1 mechanical rev = MOTOR_POLE_PAIRS * 1 electrical rev
    //                   = MOTOR_POLE_PAIRS * HALL_EDGES_PER_ELEC_REV * transition_period
    // RPM = (60 * 1,000,000) / (MOTOR_POLE_PAIRS * HALL_EDGES_PER_ELEC_REV * elapsed_ticks)
    
    float raw_rpm = (60.0f * 1000000.0f) / 
        ((float)MOTOR_POLE_PAIRS * (float)HALL_EDGES_PER_ELEC_REV * (float)elapsed_ticks);
        
    // Insert into moving average filter
    rpm_history[filter_index] = (uint16_t)raw_rpm;
    filter_index = (filter_index + 1) % FILTER_SAMPLES;
    
    // Compute moving average
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        sum += rpm_history[i];
    }
    current_rpm = (float)sum / (float)FILTER_SAMPLES;
}

/**
 * @brief Get the current filtered mechanical RPM
 * @retval Speed in RPM
 */
uint16_t Speed_GetRPM(void)
{
    return (uint16_t)current_rpm;
}
