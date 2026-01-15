/*
 * control_remote.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */


#include "board_lib.h"
#include "control_remote.h"
#include "control_input_task.h"
#include "imu_processing_task.h"
#include "motor_config.h"
#include "motor_control.h"

void remote_control_input() {
	remote_gimbal_input();
	remote_chassis_input();
	remote_launcher_control_input();
}

void remote_chassis_input() {
	if (g_remote_cmd.sw != SW_ALL_ON) {
		chassis_ctrl_data.enabled = 0;
		chassis_ctrl_data.forward = 0;
		chassis_ctrl_data.horizontal = 0;
		chassis_ctrl_data.yaw = 0;
	} else {
		chassis_ctrl_data.enabled = 1;
		float horizontal_input = 0.0;
		float forward_input = 0.0;
		float yaw_input = 0.0;

		forward_input = (float) g_remote_cmd.left_y / RC_LIMITS;
		horizontal_input = (float) g_remote_cmd.left_x / RC_LIMITS;
		if (abs(g_remote_cmd.side_dial) > 50 ){
			yaw_input = (float)g_remote_cmd.side_dial * CHASSIS_SPINSPIN_MAX/660;
		}
		else {
			yaw_input = chassis_center_yaw();
		}
		chassis_set_ctrl(forward_input, horizontal_input, yaw_input);
	}
}

void remote_gimbal_input() {
	if (g_remote_cmd.sw == SW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;

		float pitch_remote = ((float) g_remote_cmd.right_y / 660) * PITCH_INVERT
				* REMOTE_PITCH_SPEED;
		float yaw_remote = ((float) g_remote_cmd.right_x / 660) * YAW_INVERT
				* REMOTE_YAW_SPEED;
		gimbal_turn_ang(pitch_remote, yaw_remote);
	}
}


void remote_launcher_control_input() {
	launcher_ctrl_data.enabled = 1;

	if (g_remote_cmd.trigger == BUTTON_PRESSED) {
		launcher_ctrl_data.firing = 1;
	} else {
		launcher_ctrl_data.firing = 0;
	}
}

