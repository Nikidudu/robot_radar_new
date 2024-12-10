#include <math.h>
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "balancing_imu_task.h"
#include "dm4310_drv.h"

extern float imu_test[6];
orientation_data_t balancing_imu;

// Define the sample rate
#define SAMPLE_RATE_HZ 200.0f   // 5ms -> 1/0.005 = 200 Hz
#define COMPLEMENTARY_ALPHA 0.98f // Complementary filter constant (adjust as needed)

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

    const float dt = 1.0f / SAMPLE_RATE_HZ; // Time step in seconds
    const float rad_to_deg = 180.0f / (float)M_PI;

    static float pitch = 0.0f;
    static float roll = 0.0f;
    static float yaw = 0.0f;
    static int initialized = 0;

    // Compute roll and pitch from accelerometer
    float acc_roll = atan2f(accY_raw, accZ_raw); // Roll angle from accelerometer
    float acc_pitch = atan2f(-accX_raw, sqrtf(accY_raw * accY_raw + accZ_raw * accZ_raw)); // Pitch angle from accelerometer

    if (!initialized) {
        // Initialize roll, pitch, and yaw based on accelerometer data
        roll = acc_roll;
        pitch = acc_pitch;
        yaw = 0.0f;
        initialized = 1;
    }

    // Complementary filter for roll and pitch
    roll = COMPLEMENTARY_ALPHA * (roll + gyroX_raw * dt) + (1.0f - COMPLEMENTARY_ALPHA) * acc_roll;
    pitch = COMPLEMENTARY_ALPHA * (pitch + gyroY_raw * dt) + (1.0f - COMPLEMENTARY_ALPHA) * acc_pitch;

    // Integrate yaw from gyroZ
    yaw += gyroZ_raw * dt;

    // Assign radian values
    orientation->pit = -pitch;
    orientation->rol = roll;
    orientation->yaw = yaw;

    // Convert to degrees
    orientation->pit_deg = -pitch * rad_to_deg;
    orientation->rol_deg = roll * rad_to_deg;
    orientation->yaw_deg = yaw * rad_to_deg;

    // Calculate angular velocities (rad/s and deg/s)
    orientation->pit_speed = -gyroY_raw;  // Pitch angular velocity
    orientation->rol_speed = gyroX_raw;  // Roll angular velocity
    orientation->yaw_speed = gyroZ_raw;  // Yaw angular velocity

    orientation->pit_speed_deg = -gyroY_raw * rad_to_deg;
    orientation->rol_speed_deg = gyroX_raw * rad_to_deg;
    orientation->yaw_speed_deg = gyroZ_raw * rad_to_deg;
}

void balancing_imu_task(void *argument) {
    while (1) {
        sensor_fusion(imu_test, &balancing_imu);
        vTaskDelay(5); // 5 ms delay to match SAMPLE_RATE_HZ
    }
}
