#include "pid.h"

/**
 * @brief Initialize the PID Controller parameters and limit clamping
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->OutputMin = out_min;
    pid->OutputMax = out_max;
    pid->IntegralMin = out_min; // Clamping integral term to output range
    pid->IntegralMax = out_max;
    
    PID_Reset(pid);
}

/**
 * @brief Reset PID error terms and accumulated integral values
 */
void PID_Reset(PID_Controller *pid)
{
    pid->Integral = 0.0f;
    pid->PreviousError = 0.0f;
}

/**
 * @brief Compute the PID control output
 * @param pid Pointer to PID_Controller structure
 * @param setpoint Desired target speed
 * @param measured Actual measured speed
 * @param dt Time difference in seconds
 * @retval Output value limited within [OutputMin, OutputMax]
 */
float PID_Update(PID_Controller *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f) {
        dt = 0.02f; // Default back up to 20ms period
    }
    
    float error = setpoint - measured;
    
    // Proportional term
    float p_term = pid->Kp * error;
    
    // Integral term with anti-windup clamping
    pid->Integral += pid->Ki * error * dt;
    if (pid->Integral > pid->IntegralMax) {
        pid->Integral = pid->IntegralMax;
    } else if (pid->Integral < pid->IntegralMin) {
        pid->Integral = pid->IntegralMin;
    }
    
    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = pid->Kd * ((error - pid->PreviousError) / dt);
    }
    pid->PreviousError = error;
    
    // Total raw output
    float output = p_term + pid->Integral + d_term;
    
    // Output saturation limit
    if (output > pid->OutputMax) {
        output = pid->OutputMax;
    } else if (output < pid->OutputMin) {
        output = pid->OutputMin;
    }
    
    return output;
}
