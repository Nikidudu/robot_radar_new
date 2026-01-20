/*
 * imu_processing_task.h
 *
 *  Created on: 24 Jan 2022
 *      Author: wx
 */

#ifndef TASKS_INC_IMU_PROCESSING_TASK_H_
#define TASKS_INC_IMU_PROCESSING_TASK_H_

typedef struct
{
	float gx;
	float gy;
	float gz;
	uint32_t last_gyro_update;
} gyro_data_t;

typedef struct
{
	float ax;
	float ay;
	float az;
	uint32_t last_accel_update;
} accel_data_t;

typedef struct
{
	int16_t mx;
	int16_t my;
	int16_t mz;
	uint32_t last_mag_update;
} mag_data_t;

typedef struct
{
	gyro_data_t gyro_data;
	accel_data_t accel_data;
	mag_data_t mag_data;

	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} imu_raw_t;

typedef struct{
	float ax;
	float ay;
	float az;
} linear_accel_t;

extern orientation_data_t imu_heading;

void imu_processing_task(void *argument);
void gyro_data_ready(gyro_data_t gyro_data);
void accel_data_ready(accel_data_t accel_data);
void mag_data_ready(mag_data_t mag_data);

#endif /* TASKS_INC_IMU_PROCESSING_TASK_H_ */
