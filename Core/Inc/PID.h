#ifndef PID_H
#define PID_H

typedef struct {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float dt; // Time step
    float integral; // Integral term
    float previous_error; // Previous error for derivative calculation
    float anti_windup; // Anti-windup term to prevent integral windup
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd, float dt);
float PID_Compute(PID *pid, float setpoint, float measured_value);

void PID_Reset(PID *pid);
void PID_SetGains(PID *pid, float kp, float ki, float kd);
void PID_SetTimeStep(PID *pid, float dt);

#endif // PID_H