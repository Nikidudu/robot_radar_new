/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     Inertial Navigation System (INS) task implementation
  * @details   Implements Mahony filter for attitude estimation and motion 
  *           acceleration calculation in both body and earth frames
  *********************************************************************
  */
	
#include "ins_task.h"
//#include "QuaternionEKF.h"
#include "mahony_filter.h"
#include <math.h>
#include "board_lib.h"

// Global INS data structure
INS_t INS;

// Mahony filter instance and sensor data structures
struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;

// Gravity vector in earth frame (m/s^2)
float gravity[3] = {0, 0, 9.81f};

float ins_dt = 0.0f;        // Time step for integration
float ins_time;             // System runtime counter
extern float imu_test[6];   // External IMU test data

/**
 * @brief  Initialize INS parameters and Mahony filter
 * @param  None
 * @retval None
 */
void INS_Init(void)
{
    // Initialize Mahony filter with gains
    mahony_init(&mahony,1.0f,0.0f,0.005f);
    INS.AccelLPF = 0.089f;  // Set accelerometer low-pass filter coefficient
}

/**
 * @brief  Main INS task function
 * @param  argument: Not used
 * @retval None
 */
void INS_task(void *argument)
{
    INS_Init();

    while(1)
    {
        ins_dt = 0.005f;  // 5ms update rate
        mahony.dt = ins_dt;

        // Update IMU data from test array
        INS.Accel[0] = imu_test[0];
        INS.Accel[1] = imu_test[1];
        INS.Accel[2] = imu_test[2];
        Accel.x = imu_test[0];
        Accel.y = imu_test[1];
        Accel.z = imu_test[2];
        INS.Gyro[0] = imu_test[3];
        INS.Gyro[1] = imu_test[4];
        INS.Gyro[2] = -imu_test[5];
        Gyro.x = imu_test[3];
        Gyro.y = imu_test[4];
        Gyro.z = imu_test[5];

        // Update Mahony filter
        mahony_input(&mahony,Gyro,Accel);
        mahony_update(&mahony);
        mahony_output(&mahony);

        // Normalize quaternion
        float norm = sqrtf(mahony.q0 * mahony.q0 + mahony.q1 * mahony.q1 + 
                          mahony.q2 * mahony.q2 + mahony.q3 * mahony.q3);
        mahony.q0 /= norm;
        mahony.q1 /= norm;
        mahony.q2 /= norm;
        mahony.q3 /= norm;

        RotationMatrix_update(&mahony);

        // Update INS quaternion
        INS.q[0] = mahony.q0;
        INS.q[1] = mahony.q1;
        INS.q[2] = mahony.q2;
        INS.q[3] = mahony.q3;

        // Transform gravity from earth frame to body frame
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);

        // Calculate motion acceleration in body frame with low-pass filter
        for (uint8_t i = 0; i < 3; i++)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt)
                                  + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt);
        }

        // Transform motion acceleration back to earth frame
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

        // Apply acceleration deadband
        if(fabsf(INS.MotionAccel_n[0]) < 0.02f)
        {
            INS.MotionAccel_n[0] = 0.0f;  // X-axis
        }
        if(fabsf(INS.MotionAccel_n[1]) < 0.02f)
        {
            INS.MotionAccel_n[1] = 0.0f;  // Y-axis
        }
        if(fabsf(INS.MotionAccel_n[2]) < 0.04f)
        {
            INS.MotionAccel_n[2] = 0.0f;  // Z-axis
        }

        // Update attitude angles after initialization period
        if(ins_time > 100.0f)
        {
            INS.ins_flag = 1;  // Set initialization complete flag
            
            // Update Euler angles
            INS.Pitch = -mahony.pitch;
            INS.Roll = mahony.roll;
            INS.Yaw = mahony.yaw;

            // Calculate total yaw angle including rotations
            if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
            {
                INS.YawRoundCount--;
            }
            else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
            {
                INS.YawRoundCount++;
            }
            INS.YawTotalAngle = 6.283f * INS.YawRoundCount + INS.Yaw;
            INS.YawAngleLast = INS.Yaw;
        }
        else
        {
            ins_time++;  // Increment initialization counter
        }

        osDelay(5);  // 5ms task period
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}




