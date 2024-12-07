#include <math.h>
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "balancing_imu_task.h"
#include "dm4310_drv.h"

extern float imu_test[6];
orientation_data_t balancing_imu;

// Define the cutoff frequency and sample rate
#define SAMPLE_RATE_HZ 200.0f   // 5ms -> 1/0.005 = 200 Hz
#define CUTOFF_FREQ_HZ 20.0f

// Compute the filter coefficient 'a' for first-order Butterworth
static const float a = 0.0f; // We'll compute this in init section below

// Initialize a and static states for the filters
static float a_coeff;

static float accX_filtered = 0.0f;
static float accY_filtered = 0.0f;
static float accZ_filtered = 0.0f;
static float gyroX_filtered = 0.0f;
static float gyroY_filtered = 0.0f;
static float gyroZ_filtered = 0.0f;

static int filter_initialized = 0;

static void init_filter(void) {
    if (!filter_initialized) {
        float rc = 1.0f / (2.0f * (float)M_PI * CUTOFF_FREQ_HZ);
        float dt = 1.0f / SAMPLE_RATE_HZ;
        // For a first-order low-pass butterworth, a simpler expression:
        // a = e^(-2*pi*fc/fs)
        a_coeff = expf(-2.0f * (float)M_PI * CUTOFF_FREQ_HZ / SAMPLE_RATE_HZ);
        filter_initialized = 1;
    }
}

// A generic function to apply a first-order LPF
static float butterworth_filter(float x, float prev_y) {
    // y[k] = a * y[k-1] + (1 - a)* x[k]
    float y = a_coeff * prev_y + (1.0f - a_coeff) * x;
    return y;
}

static void sensor_fusion(const float imu_data[6], orientation_data_t *orientation)
{
    // imu_data is:
    // [0]: Accel X (m/s²)
    // [1]: Accel Y (m/s²)
    // [2]: Accel Z (m/s²)
    // [3]: Gyro X (rad/s)
    // [4]: Gyro Y (rad/s)
    // [5]: Gyro Z (rad/s)

    float accX_raw = imu_data[0];
    float accY_raw = imu_data[1];
    float accZ_raw = imu_data[2];
    float gyroX_raw = imu_data[3];  // rad/s (roll rate)
    float gyroY_raw = imu_data[4];  // rad/s (pitch rate)
    float gyroZ_raw = imu_data[5];  // rad/s (yaw rate)

    // Apply Butterworth filter to each channel
    accX_filtered = butterworth_filter(accX_raw, accX_filtered);
    accY_filtered = butterworth_filter(accY_raw, accY_filtered);
    accZ_filtered = butterworth_filter(accZ_raw, accZ_filtered);
    gyroX_filtered = butterworth_filter(gyroX_raw, gyroX_filtered);
    gyroY_filtered = butterworth_filter(gyroY_raw, gyroY_filtered);
    gyroZ_filtered = butterworth_filter(gyroZ_raw, gyroZ_filtered);

    float accX = accX_filtered;
    float accY = accY_filtered;
    float accZ = accZ_filtered;
    float gyroX = gyroX_filtered;
    float gyroY = gyroY_filtered;
    float gyroZ = gyroZ_filtered;

    const float dt = 0.005f;     // 5 ms
    const float alpha = 0.98f;   // Complementary filter constant
    const float rad_to_deg = 180.0f / (float)M_PI;

    // Filter constants for angular velocities (existing low-pass)
    const float speed_alpha = 0.9f;

    static float pitch = 0.0f;
    static float roll = 0.0f;
    static float yaw = 0.0f;
    static int initialized = 0;

    // Low-pass filtered gyro values
    static float filtered_gyroX = 0.0f;
    static float filtered_gyroY = 0.0f;
    static float filtered_gyroZ = 0.0f;

    // Compute roll and pitch from accelerometer
    float acc_roll = atan2f(accY, accZ);
    float acc_pitch = atan2f(accX, sqrtf(accY*accY + accZ*accZ));

    if (!initialized) {
        roll = acc_roll;
        pitch = acc_pitch;
        yaw = 0.0f;
        initialized = 1;
    }

    // Complementary filter for roll and pitch
    roll = alpha * (roll + gyroX * dt) + (1.0f - alpha) * acc_roll;
    pitch = alpha * (pitch + gyroY * dt) + (1.0f - alpha) * acc_pitch;

    // Integrate yaw from gyroZ
    yaw += gyroZ * dt;

    // Additional low-pass for gyro speeds
    filtered_gyroX = speed_alpha * filtered_gyroX + (1.0f - speed_alpha) * gyroX;
    filtered_gyroY = speed_alpha * filtered_gyroY + (1.0f - speed_alpha) * gyroY;
    filtered_gyroZ = speed_alpha * filtered_gyroZ + (1.0f - speed_alpha) * gyroZ;

    // Correct pitch speed sign if needed
    float corrected_pit_speed = -filtered_gyroY;

    // Assign radian values
    orientation->pit = pitch;
    orientation->rol = roll;
    orientation->yaw = yaw;
    orientation->pit_speed = corrected_pit_speed;
    orientation->rol_speed = filtered_gyroX;
    orientation->yaw_speed = filtered_gyroZ;

    // Convert to degrees
    orientation->pit_deg = pitch * rad_to_deg;
    orientation->rol_deg = roll * rad_to_deg;
    orientation->yaw_deg = yaw * rad_to_deg;
    orientation->pit_speed_deg = corrected_pit_speed * rad_to_deg;
    orientation->rol_speed_deg = filtered_gyroX * rad_to_deg;
    orientation->yaw_speed_deg = filtered_gyroZ * rad_to_deg;
}

void balancing_imu_task(void *argument) {
    init_filter();

    while (1) {
        sensor_fusion(imu_test, &balancing_imu);
        vTaskDelay(5);
        MFtorque_command(&hcan2, 0x141, -20);
        MFtorque_command(&hcan2, 0x142, 20);
    }
}
