#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Integral;
    float PreviousError;
    
    float OutputMin;
    float OutputMax;
    float IntegralMin;
    float IntegralMax;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max);
void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float setpoint, float measured, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
