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
#define DT (1.0f / SAMPLE_RATE_HZ)

// Kalman filter parameters
// You may need to tune these values for your specific IMU and noise levels.
#define Q_ANGLE  0.0001f   // Process noise variance for the angle
#define Q_BIAS   0.001f   // Process noise variance for the gyro bias
#define R_MEAS   0.1f    // Measurement noise variance (accelerometer)

typedef struct {
    float angle;
    float bias;
    float P[2][2];
} kalman_filter_t;

static kalman_filter_t kf_roll;
static kalman_filter_t kf_pitch;
static int initialized = 0;

// Initialize a Kalman filter structure
static void kalman_filter_init(kalman_filter_t *kf, float init_angle) {
    kf->angle = init_angle;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f; // Since we trust our initial angle guess well, we can start at 0 or a small value
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

// Kalman filter predict/update for one axis (pitch or roll)
static float kalman_filter_update(kalman_filter_t *kf, float new_angle_measure, float new_rate) {
    // Prediction step
    float rate = new_rate - kf->bias;
    kf->angle += DT * rate;

    // Update covariance P
    kf->P[0][0] += DT * (DT*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_ANGLE);
    kf->P[0][1] -= DT * kf->P[1][1];
    kf->P[1][0] -= DT * kf->P[1][1];
    kf->P[1][1] += Q_BIAS * DT;

    // Measurement update
    float S = kf->P[0][0] + R_MEAS; // S = P[0][0] + R
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float y = new_angle_measure - kf->angle; // Angle difference

    // Update state with measurement
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    // Update P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
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
    float gyroX_raw = imu_data[3];  // roll rate
    float gyroY_raw = imu_data[4];  // pitch rate
    float gyroZ_raw = imu_data[5];  // yaw rate

    const float rad_to_deg = 180.0f / (float)M_PI;

    // Compute raw angles from accelerometer
    float acc_roll = atan2f(accY_raw, accZ_raw);
    float acc_pitch = atan2f(-accX_raw, sqrtf(accY_raw * accY_raw + accZ_raw * accZ_raw));

    if (!initialized) {
        // Initialize Kalman filters for roll and pitch
        kalman_filter_init(&kf_roll, acc_roll);
        kalman_filter_init(&kf_pitch, acc_pitch);
        initialized = 1;
    }

    // Kalman filter update for roll
    float roll = kalman_filter_update(&kf_roll, acc_roll, gyroX_raw);

    // Kalman filter update for pitch
    float pitch = kalman_filter_update(&kf_pitch, acc_pitch, gyroY_raw);

    // Integrate yaw from gyro (no absolute reference)
    static float yaw = 0.0f;
    yaw += gyroZ_raw * DT;

    // Assign radian values (keep directions same as original code)
    orientation->pit = -pitch;
    orientation->rol = roll;
    orientation->yaw = yaw;

    // Convert to degrees
    orientation->pit_deg = -pitch * rad_to_deg;
    orientation->rol_deg = roll * rad_to_deg;
    orientation->yaw_deg = yaw * rad_to_deg;

    // Angular velocities (rad/s and deg/s) remain unchanged
    orientation->pit_speed = -gyroY_raw;  // Pitch angular velocity
    orientation->rol_speed = gyroX_raw;   // Roll angular velocity
    orientation->yaw_speed = gyroZ_raw;   // Yaw angular velocity

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
