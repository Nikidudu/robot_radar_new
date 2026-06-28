/*
 * gimbal_control_task.h
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#ifndef TASKS_INC_GIMBAL_CONTROL_TASK_H_
#define TASKS_INC_GIMBAL_CONTROL_TASK_H_

extern motor_data_t yaw_motor;
extern motor_data_t pitch_motor;

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

void gimbal_control_task(void *argument);
#endif /* TASKS_INC_GIMBAL_CONTROL_TASK_H_ */
