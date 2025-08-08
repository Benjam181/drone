#include "PID.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float dt, float u_min, float u_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->anti_windup = 100.0f; // Initialize anti-windup term
    pid->u_min = u_min; // Minimum output value
    pid->u_max = u_max; // Maximum output value
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
    return fminf((fmaxf(proportional + pid->integral + derivative, pid->u_min)), pid->u_max);
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

int32_t low_pass_filter(int32_t input, int32_t previous_output, float alpha) {
    // Apply a simple low-pass filter
    return (int32_t)(alpha * input + (1.0f - alpha) * previous_output);
}
