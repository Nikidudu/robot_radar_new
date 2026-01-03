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

extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t aimbot_mode;

void control_input_task(void *argument);
float chassis_center_yaw();
void chassis_centering_config();
void chassis_set_ctrl(float forward, float horizontal, float yaw);
void chassis_kill_ctrl();
void control_reset();
void control_mode_change(uint8_t control_mode, uint8_t fn_1);
//ADDs angle to gimbal ctrl
void gimbal_turn_ang(float pit_radians, float yaw_radians);
//SETs angle to gimbal ctrl
void gimbal_set_ang(float pit_radians, float yaw_radians);
void chassis_yaw_pid_init();
void ramp(float *curr_val, float target_val, float max_ramp);

extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern gimbal_control_t gimbal_ctrl_data;

#ifdef __cplusplus
}
#endif

#endif /* TASKS_INC_CONTROL_INPUT_TASK_H_ */
