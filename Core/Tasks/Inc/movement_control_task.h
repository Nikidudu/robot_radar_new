/*
 * movement_control_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_MOVEMENT_CONTROL_TASK_H_
#define TASKS_INC_MOVEMENT_CONTROL_TASK_H_

void movement_control_task(void *argument);
void chassis_motion_control();
void chassis_pid_init();
void level_config(uint32_t *lvl_max_speed, uint32_t *lvl_max_accel, int8_t *lvl_max_spin);
void rpm_ramp(int16_t *rpms, int16_t* current_rpm, uint32_t chassis_rpm, uint32_t lvl_max_accel);
void yaw_zeroing(motor_data_t *motorfr, motor_data_t *motorfl, motor_data_t *motorbl, motor_data_t *motorbr);

#endif /* TASKS_INC_MOVEMENT_CONTROL_TASK_H_ */
