/*
 * launcher_control_task.h
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_LAUNCHER_CONTROL_TASK_H_
#define TASKS_INC_LAUNCHER_CONTROL_TASK_H_


uint16_t check_overheat();

void launcher_control_task(void *argument);
void flywheel_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel);
void launcher_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,motor_data_t *feeder);
void launcher_angle_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,motor_data_t *feeder);

void guidance_flywheel(motor_data_t *l_flywheel, motor_data_t *r_flywheel, motor_data_t *b_flywheel);
void guidance_feeder(motor_data_t *l_flywheel, motor_data_t *r_flywheel, motor_data_t *b_flywheel,
		motor_data_t *g_flywheel, motor_data_t *feeder);
int32_t flywheel_ramp(int32_t target_value, int32_t current_value, int32_t ramp_rate);

#endif /* TASKS_INC_LAUNCHER_CONTROL_TASK_H_ */
