#include <math.h>
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "balancing_imu_task.h"

extern float imu_test[6];
orientation_data_t balancing_imu;

static void sensor_fusion(const float imu_data[6], orientation_data_t *orientation)
{
    // imu_data is assumed to be:
    // imu_data[0]: Accel X in m/s²
    // imu_data[1]: Accel Y in m/s²
    // imu_data[2]: Accel Z in m/s²
    // imu_data[3]: Gyro X in rad/s
    // imu_data[4]: Gyro Y in rad/s
    // imu_data[5]: Gyro Z in rad/s

    float accX = imu_data[0];
    float accY = imu_data[1];
    float accZ = imu_data[2];
    float gyroX = imu_data[3];  // rad/s (roll rate)
    float gyroY = imu_data[4];  // rad/s (pitch rate)
    float gyroZ = imu_data[5];  // rad/s (yaw rate)

    const float dt = 0.005f;     // 5 ms
    const float alpha = 0.98f;   // Complementary filter constant
    const float rad_to_deg = 180.0f / (float)M_PI;

    // Filter constants for angular velocities
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
    // roll = atan2(Y, Z)
    // pitch = atan2(X, sqrt(Y² + Z²))
    // (Using accX positive for pitch; if inverted, add a minus sign before accX)
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

    // Apply low-pass filtering to gyro readings to smooth out speeds
    filtered_gyroX = speed_alpha * filtered_gyroX + (1.0f - speed_alpha) * gyroX;
    filtered_gyroY = speed_alpha * filtered_gyroY + (1.0f - speed_alpha) * gyroY;
    filtered_gyroZ = speed_alpha * filtered_gyroZ + (1.0f - speed_alpha) * gyroZ;

    // If pitch speed seems inverted, invert it here:
    // This ensures that if pitch angle increases positively, pitch speed is also positive.
    float corrected_pit_speed = -filtered_gyroY;

    // Assign radian values
    orientation->pit = pitch;
    orientation->rol = roll;
    orientation->yaw = yaw;
    orientation->pit_speed = corrected_pit_speed; // corrected sign for pitch speed
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
    while (1) {
        sensor_fusion(imu_test, &balancing_imu);
        vTaskDelay(5);
    }
}
