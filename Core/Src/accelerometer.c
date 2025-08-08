#include "accelerometer.h"

#define MPU6050_ADDR (0x68 << 1) // Adresse I2C en 8 bits

/**
 * @brief Initializes the accelerometer hardware and configures its settings.
 *
 * This function sets up the accelerometer by configuring the necessary hardware
 * registers, communication interfaces (such as I2C or SPI), and any required
 * initialization parameters. It prepares the accelerometer for data acquisition
 * and ensures it is ready for operation.
 *
 * @note This function should be called before attempting to read data from the accelerometer.
 */
void Accelerometer_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data = 0;
    uint8_t check;
    // Read WHO_AM_I register (0x75) to check if the device is connected
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check != 0x68) {
        // Device not found, handle error
        return;
    }
    // PWR_MGMT_1 Register (0x6B) : wake-up
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
    // Set DATA RATE of 1kHz by writing SMPLRT_DIV register
    data = 0x07; // 1000 Hz / (1 + 7) = 125 Hz
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, 1000);
    // Set accelerometer configuration in ACCEL_CONFIG Register (0x1C)
    data = 0x00; // ±2g full scale
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);
    // set Gyroscopic configuration in GYRO_CONFIG Register (0x1B)
    data = 0x00; // ±250°/s full scale
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
}

/**
 * @brief Reads the current acceleration values from the accelerometer sensor.
 *
 * This function communicates with the accelerometer hardware to retrieve the latest
 * acceleration data, typically along the X, Y, and Z axes. The retrieved values can be
 * used for motion detection, orientation sensing, or other applications requiring
 * acceleration information.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
void Accelerometer_Read(I2C_HandleTypeDef *hi2c, AccelData *accel_data, GyroData *gyro_data)
{    
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) Register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);
    // Convert the data to 16-bit signed integers
    accel_data->x = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);
    accel_data->y = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);
    accel_data->z = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);
    // Convert raw value to g-force
    accel_data->x = (float)accel_data->x / 16384.0f; // For ±2g full scale
    accel_data->y = (float)accel_data->y / 16384.0f; 
    accel_data->z = (float)accel_data->z / 16384.0f;

    // Read 6 BYTES of data starting from GYRO_XOUT_H (0x43) Register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);
    // Convert the data to 16-bit signed integers
    gyro_data->x = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);
    gyro_data->y = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);
    gyro_data->z = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);
    // Convert raw value to degrees per second
    gyro_data->x = (float)gyro_data->x / 131.0f; // For ±250°/s full scale
    gyro_data->y = (float)gyro_data->y / 131.0f;
    gyro_data->z = (float)gyro_data->z / 131.0f; 
}

/**
 * @brief Calculates the pitch and roll angles from accelerometer data.
 *
 * This function computes the pitch and roll angles based on the accelerometer's
 * x, y, and z axis readings. The angles are calculated using trigonometric functions
 * to determine the orientation of the device in space.
 *
 * @param accel_data Structure containing accelerometer data (x, y, z).
 * @param pitch Pointer to store the calculated pitch angle (in degrees).
 * @param roll Pointer to store the calculated roll angle (in degrees).
 */
void angles_from_accel(AccelData accel_data, float *pitch, float *roll) {
    *pitch = atan2f(accel_data.y, sqrtf(accel_data.x * accel_data.x + accel_data.z * accel_data.z)) * (180.0f / M_PI);
    *roll = atan2f(-accel_data.x, sqrtf(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * (180.0f / M_PI);
}

/**
 * @brief Calculates the pitch and roll rates from gyroscope data.
 *
 * This function computes the pitch and roll rates based on the gyroscope's
 * x and y axis readings. The rates are derived directly from the gyroscope data,
 * which provides angular velocity measurements.
 *
 * @param gyro_data Structure containing gyroscope data (x, y, z).
 * @param pitch_rate Pointer to store the calculated pitch rate (in degrees per second).
 * @param roll_rate Pointer to store the calculated roll rate (in degrees per second).
 */
void angles_rate_from_gyro(GyroData gyro_data, float *pitch_rate, float *roll_rate, float *yaw_rate) {
    *pitch_rate = gyro_data.y; // Gyro Y-axis for pitch rate
    *roll_rate = gyro_data.x;  // Gyro X-axis for roll rate
    *yaw_rate = gyro_data.z;   // Gyro Z-axis for yaw rate
}

/**
 * @brief Initializes the Kalman filter state.
 *
 * This function sets up the initial parameters for the Kalman filter, including
 * the initial state estimate, error covariance, process noise, and measurement noise.
 * It should be called before using the filter for prediction or update steps.
 *
 * @param kf Pointer to the Kalman filter structure to initialize.
 * @param init_state Initial estimate of the state.
 * @param init_covariance Initial estimate of the error covariance.
 * @param process_noise Process noise covariance (Q).
 * @param measurement_noise Measurement noise covariance (R).
 */
void Kalman_Init(Kalman_t *kalman_x, Kalman_t *kalman_y)
{
    // Initialize Kalman filter parameters for X, Y, and Z axes
    kalman_x->angle = 0.0f;
    kalman_x->bias = 0.0f;
    kalman_x->rate = 0.0f;
    kalman_x->P[0][0] = 1.0f; // P[0][0]
    kalman_x->P[0][1] = 0.0f; // P[0][1]
    kalman_x->P[1][0] = 0.0f; // P[1][0]
    kalman_x->P[1][1] = 1.0f; // P[1][1]
    kalman_x->Q_angle = 0.001f; // Process noise covariance for angle
    kalman_x->Q_bias = 0.003f; // Process noise covariance for bias
    kalman_x->R_measure = 0.03f; // Measurement noise covariance

    kalman_y->angle = 0.0f;
    kalman_y->bias = 0.0f;
    kalman_y->rate = 0.0f;
    kalman_y->P[0][0] = 1.0f;
    kalman_y->P[0][1] = 0.0f;
    kalman_y->P[1][0] = 0.0f;
    kalman_y->P[1][1] = 1.0f;
    kalman_y->Q_angle = 0.001f;
    kalman_y->Q_bias = 0.003f;
    kalman_y->R_measure = 0.03f;
}

/**
 * @brief Computes the filtered angle using the Kalman filter algorithm.
 *
 * This function estimates the true angle by combining accelerometer and gyroscope data,
 * reducing noise and drift. It performs the prediction and update steps of the Kalman filter
 * to provide a more accurate and stable angle estimate. The kalman filter works for the pitch and the roll axis.
 *
 * @param kalman Pointer to the Kalman filter structure for the axis.
 * @param acc_angle Angle measured from the accelerometer (in degrees).
 * @param gyro_rate Angular rate measured from the gyroscope (in degrees per second).
 * @param dt Time interval since the last update (in seconds).
 */
void Kalman_GetAngle(Kalman_t *kalman, float acc_angle, float gyro_rate, float dt) {
    // 1. Prediction
    kalman->rate = gyro_rate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // state covariance update
    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // 2. update
    float y = acc_angle - kalman->angle; // innovation
    float S = kalman->P[0][0] + kalman->R_measure; // incertitude innovation
    float K0 = kalman->P[0][0] / S;
    float K1 = kalman->P[1][0] / S;

    // state update
    kalman->angle += K0 * y;
    kalman->bias  += K1 * y;

    // state covariance update
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K0 * P00_temp;
    kalman->P[0][1] -= K0 * P01_temp;
    kalman->P[1][0] -= K1 * P00_temp;
    kalman->P[1][1] -= K1 * P01_temp;
}
