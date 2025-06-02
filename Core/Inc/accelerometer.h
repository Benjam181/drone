#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <math.h>

#define M_PI		3.14159265358979323846

// Structure pour stocker les valeurs d'accélération
typedef struct {
    float x;
    float y;
    float z;
} AccelData;

typedef struct {
    float x;
    float y;
    float z;
}GyroData;

typedef struct {
    float angle;
    float bias;
    float rate; // corrected speed (gyro - bias)
    float P[2][2]; // covariance matrix
    float Q_angle; // process noise covariance for angle
    float Q_bias; // process noise covariance for bias
    float R_measure; // measurement noise covariance
} Kalman_t;

// Initialisation de l'accéléromètre
void Accelerometer_Init(I2C_HandleTypeDef *hi2c);

// Lecture des données d'accélération
void Accelerometer_Read(I2C_HandleTypeDef *hi2c, AccelData *data, GyroData *gyro_data);

void angles_from_accel(AccelData accel_data, float *pitch, float *roll);
void angles_rate_from_gyro(GyroData gyro_data, float *pitch_rate, float *roll_rate);

void Kalman_Init(Kalman_t *kalman_x, Kalman_t *kalman_y);
void Kalman_GetAngle(Kalman_t *kalman, float acc_angle, float gyro_rate, float dt);

#endif // ACCELEROMETER_H