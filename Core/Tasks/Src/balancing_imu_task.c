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
// General parameters
#define Q_ANGLE  0.00003f   // Balance between smoothness and response
#define Q_BIAS   0.001f     // Moderate adaptation to gyro bias
#define R_MEAS   0.6f

// Roll-specific parameters
#define Q_ANGLE_ROLL  0.00003f
#define Q_BIAS_ROLL   0.0001f
#define R_MEAS_ROLL   0.6f

typedef struct {
    float angle;
    float bias;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_meas;
} kalman_filter_t;

static kalman_filter_t kf_roll;
static kalman_filter_t kf_pitch;
static int initialized = 0;

// Initialize a Kalman filter structure with specific parameters
static void kalman_filter_init(kalman_filter_t *kf, float init_angle, float Q_angle, float Q_bias, float R_meas) {
    kf->angle = init_angle;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_meas = R_meas;
}

// Kalman filter predict/update for one axis (pitch or roll)
static float kalman_filter_update(kalman_filter_t *kf, float new_angle_measure, float new_rate) {
    // Prediction step
    float rate = new_rate - kf->bias;
    kf->angle += DT * rate;

    // Update covariance P
    kf->P[0][0] += DT * (DT * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= DT * kf->P[1][1];
    kf->P[1][0] -= DT * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * DT;

    // Measurement update
    float S = kf->P[0][0] + kf->R_meas;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float y = new_angle_measure - kf->angle; // Angle difference

    // Update state with measurement
    kf->angle += K0 * y;
    kf->bias += K1 * y;

    // Update P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
}

static void sensor_fusion(const float imu_data[6], orientation_data_t *orientation) {
    static float accX_filtered = 0.0f, accY_filtered = 0.0f, accZ_filtered = 0.0f;
    const float alpha = 0.8f;

    float accX_raw = imu_data[0];
    float accY_raw = imu_data[1];
    float accZ_raw = imu_data[2];
    float gyroX_raw = imu_data[3];
    float gyroY_raw = imu_data[4];
    float gyroZ_raw = imu_data[5];

    accX_filtered = alpha * accX_filtered + (1.0f - alpha) * accX_raw;
    accY_filtered = alpha * accY_filtered + (1.0f - alpha) * accY_raw;
    accZ_filtered = alpha * accZ_filtered + (1.0f - alpha) * accZ_raw;

    float acc_roll = atan2f(accY_filtered, accZ_filtered);
    float acc_pitch = atan2f(-accX_filtered, sqrtf(accY_filtered * accY_filtered + accZ_filtered * accZ_filtered));

    if (!initialized) {
        // Initialize Kalman filters for roll and pitch with respective parameters
        kalman_filter_init(&kf_roll, acc_roll, Q_ANGLE_ROLL, Q_BIAS_ROLL, R_MEAS_ROLL);
        kalman_filter_init(&kf_pitch, acc_pitch, Q_ANGLE, Q_BIAS, R_MEAS);
        initialized = 1;
    }

    // Kalman filter update for roll with roll-specific parameters
    float roll = kalman_filter_update(&kf_roll, acc_roll, gyroX_raw);

    // Kalman filter update for pitch with general parameters
    float pitch = kalman_filter_update(&kf_pitch, acc_pitch, gyroY_raw);

    // Integrate yaw from gyro (no absolute reference)
    static float yaw = 0.0f;
    yaw += gyroZ_raw * DT;
    yaw = fmodf(yaw, 2.0f * M_PI);
    if (yaw < 0) yaw += 2.0f * M_PI;

    // Assign radian values
    orientation->pit = -pitch;
    orientation->rol = roll;
    orientation->yaw = yaw;

    // Angular velocities remain unchanged
    orientation->pit_speed = -gyroY_raw;  // Pitch angular velocity
    orientation->rol_speed = gyroX_raw;   // Roll angular velocity
    orientation->yaw_speed = gyroZ_raw;   // Yaw angular velocity
}

void balancing_imu_task(void *argument) {
    while (1) {
        sensor_fusion(imu_test, &balancing_imu);
        vTaskDelay(5); // 5 ms delay to match SAMPLE_RATE_HZ
    }
}
