/*
 * imu_processing_task.h
 *
 *  Created on: 24 Jan 2022
 *      Author: wx
 */

#ifndef TASKS_INC_IMU_PROCESSING_TASK_H_
#define TASKS_INC_IMU_PROCESSING_TASK_H_

extern orientation_data_t imu_heading;

void imu_processing_task(void *argument);
void gyro_data_ready(gyro_data_t gyro_data);
void accel_data_ready(accel_data_t accel_data);
void mag_data_ready(mag_data_t mag_data);

#endif /* TASKS_INC_IMU_PROCESSING_TASK_H_ */
