#include "PID.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->anti_windup = 100.0f; // Initialize anti-windup term
}

float PID_Compute(PID *pid, float setpoint, float measured_value) {
    // Calculate error
    float error = setpoint - measured_value;

    // Proportional term
    float proportional = pid->kp * error;

    // Integral term
    pid->integral += pid->ki * error * pid->dt;

    // Anti-windup
    if (pid->integral > pid->anti_windup) {
        pid->integral = pid->anti_windup;
    } else if (pid->integral < -pid->anti_windup) {
        pid->integral = -pid->anti_windup;
    }

    // Derivative term
    float derivative = pid->kd * (error - pid->previous_error) / pid->dt;

    // Update previous error
    pid->previous_error = error;

    // Compute output
    return proportional + pid->integral + derivative;
}
void PID_Reset(PID *pid) {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}
void PID_SetGains(PID *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
void PID_SetTimeStep(PID *pid, float dt) {
    pid->dt = dt;
}
