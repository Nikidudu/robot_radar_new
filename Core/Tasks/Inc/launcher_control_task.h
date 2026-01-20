/*
 * launcher_control_task.h
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_LAUNCHER_CONTROL_TASK_H_
#define TASKS_INC_LAUNCHER_CONTROL_TASK_H_

extern motor_data_t flywheel_motor[4]; // 4 friction wheels max
extern motor_data_t feeder_motor;
extern enum feeder_state_e feeder_state;

void launcher_control_task(void *argument);

#endif /* TASKS_INC_LAUNCHER_CONTROL_TASK_H_ */
