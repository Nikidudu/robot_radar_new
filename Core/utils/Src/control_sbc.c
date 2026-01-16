/*
 * control_sbc.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "control_sbc.h"
#include "control_input_task.h"
#include "usb_task.h"

#define AIMBOT_DEADZONE 0.005f
#define AIMBOT_YAW_KP   -2.5f
#define AIMBOT_PITCH_KP 0.4f
#define FILTER_ALPHA 	0.35f  // 0.0 = no filter, 0.5 = moderate smoothing

void sbc_gimbal_input();
void sbc_launcher_control_input();

void sbc_control_input() {
	sbc_gimbal_input();
//	sbc_chassis_input();
	sbc_launcher_control_input();
}

void sbc_gimbal_input() {
    static float filtered_pitch_error = 0.0f;
    static float last_yaw_err = 0.0f; // New: for D-term
    static uint32_t last_aimbot_update_tick = 0;

    // Tuning Constants
    const float filter_alpha = 0.35f;
    const float YAW_KP = -1.8f;  // Lowered slightly to reduce initial vibration
    const float YAW_KD = -0.15f; // The "Brake" (Derivative gain)

    if (g_remote_cmd.sw == SW_SHUTDOWN) {
        gimbal_ctrl_data.enabled = 0;
        last_yaw_err = 0;
    } else {
        gimbal_ctrl_data.enabled = 1;

        float raw_yaw   = g_aimbot_cmd.yaw;
        float raw_pitch = g_aimbot_cmd.pitch;

        if (fabsf(raw_yaw) < 0.005f) raw_yaw = 0.0f;
        if (fabsf(raw_pitch) < 0.005f) raw_pitch = 0.0f;

        if (raw_yaw != 0.0f || raw_pitch != 0.0f) {
            last_aimbot_update_tick = HAL_GetTick();
        }

        float p_out = raw_yaw * YAW_KP;

        float d_out = (raw_yaw - last_yaw_err) * YAW_KD;
        last_yaw_err = raw_yaw;

        float yaw_command = p_out + d_out;

        filtered_pitch_error = filtered_pitch_error * (1.0f - filter_alpha) + raw_pitch * filter_alpha;
        float pitch_command = filtered_pitch_error;

        // Timeout Logic
        if ((HAL_GetTick() - last_aimbot_update_tick) > 300) {
            filtered_pitch_error *= 0.95f;
            yaw_command = 0.0f;
        }
        yaw_command -= 0.040f;
        pitch_command -= 0.10f;

        gimbal_set_ang(pitch_command, yaw_command);
    }
}

void sbc_launcher_control_input() {
	launcher_ctrl_data.enabled = 1;

	if (g_aimbot_cmd.fire == 1 && g_remote_cmd.trigger == 1) {
		launcher_ctrl_data.firing = 1;
	} else {
		launcher_ctrl_data.firing = 0;
	}
}
