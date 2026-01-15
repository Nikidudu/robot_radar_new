/*
 * control_keyboard.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "control_keyboard.h"
#include "control_input_task.h"
#include "chassis_can_message_task.h"
#include "imu_processing_task.h"
#include "motor_config.h"
#include "motor_control.h"

void keyboard_control_input() {
	mouse_gimbal_input();
	keyboard_chassis_input();
	mouse_launcher_control_input();
}

void keyboard_chassis_input() {
	static uint8_t shift_prev_state = 0;
	static uint32_t shift_last_change_time = 0;
	const uint32_t debounce_time = 100;  // debounce threshold in ms
	if (g_remote_cmd.sw != SW_ALL_ON) {
		chassis_ctrl_data.enabled = 0;
		chassis_ctrl_data.horizontal = 0;
		chassis_ctrl_data.forward = 0;
		chassis_ctrl_data.yaw = 0;
	} else {
		chassis_ctrl_data.enabled = 1;
		float horizontal_input = 0.0;
		float forward_input = 0.0;
		float yaw_input = 0.0;


#ifdef CHASSIS_CAN_SPINSPIN
		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_Q) {
			chassis_ctrl_data.g_spinspin_mode = 1;
		} else if (g_remote_cmd.keyboard_keys & KEY_OFFSET_E) {
			chassis_ctrl_data.g_spinspin_mode = 0;
		}
#endif

		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_W) {
			forward_input += KEYBD_MAX_SPD;
		}
		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_S) {
			forward_input -= KEYBD_MAX_SPD;
		}

		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_A) {
			horizontal_input -= KEYBD_MAX_SPD;
		}
		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_D) {
			horizontal_input += KEYBD_MAX_SPD;
		}
		if (g_remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT) {
			if (shift_prev_state == 0 && HAL_GetTick() - shift_last_change_time > debounce_time) {
				supercap.supercap_enabled ^= 1;  // toggle 0 ↔ 1
				shift_prev_state = 1;
				shift_last_change_time = HAL_GetTick();
			}
		} else {
			shift_prev_state = 0;  // reset when key is released
		}

		if (g_remote_cmd.mouse_right) {
			aimbot_mode = 1;
		} else {
			aimbot_mode = 0;
		}

		if (chassis_ctrl_data.g_spinspin_mode) {
			yaw_input = chassis_ctrl_data.g_spinspin_mode * CHASSIS_SPINSPIN_MAX;
		} else {
			//center yaw motor such that yaw motor = 0
			yaw_input = chassis_center_yaw();
		}
		chassis_set_ctrl(forward_input, horizontal_input, yaw_input);
	}
}

void mouse_gimbal_input() {
	if (g_remote_cmd.sw == SW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;

		float pitch_mouse = (float) g_remote_cmd.mouse_vert * MOUSE_Y_INVERT
				* PITCH_INVERT * MOUSE_Y_SENSITIVITY / 32768;
		float yaw_mouse = (float) g_remote_cmd.mouse_hori * MOUSE_X_INVERT
				* YAW_INVERT * MOUSE_X_SENSITIVITY / 32768;
		g_remote_cmd.mouse_vert = 0;
		g_remote_cmd.mouse_hori = 0;
		gimbal_turn_ang(pitch_mouse, yaw_mouse);
	}
}

void mouse_launcher_control_input() {
	launcher_ctrl_data.enabled = 1;

	if (g_remote_cmd.mouse_left) {
		launcher_ctrl_data.firing = 1;
	} else {
		launcher_ctrl_data.firing = 0;
	}
}
