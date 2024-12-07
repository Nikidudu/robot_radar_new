/*
 * balancing_imu_task.h
 *
 *  Created on: Dec 7, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_BALANCING_IMU_TASK_H_
#define TASKS_INC_BALANCING_IMU_TASK_H_

void balancing_imu_task(void *argument) ;
static void sensor_fusion(const float imu_data[6], orientation_data_t *orientation);



#endif /* TASKS_INC_BALANCING_IMU_TASK_H_ */
