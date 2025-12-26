/*
 * control_input_task.h
 *
 *  Created on: 4 Jul 2021
 *      Author: wx
 */

#ifndef TASKS_INC_CONTROL_INPUT_TASK_H_
#define TASKS_INC_CONTROL_INPUT_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif


void control_input_task(void *argument);
float chassis_center_yaw();
void chassis_centering_config();
void chassis_set_ctrl(float forward, float horizontal, float yaw);
void chassis_kill_ctrl();
uint8_t gimbal_aim_at_damaged_plate();
void control_reset();
void control_mode_change(uint8_t control_mode, uint8_t fn_1);
//ADDs angle to gimbal ctrl
void gimbal_turn_ang(float pit_radians, float yaw_radians);
//SETs angle to gimbal ctrl
void gimbal_set_ang(float pit_radians, float yaw_radians);
void chassis_yaw_pid_init();
void ramp(float *curr_val, float target_val, float max_ramp);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_INC_CONTROL_INPUT_TASK_H_ */
