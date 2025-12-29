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
#endif /* TASKS_INC_IMU_PROCESSING_TASK_H_ */
