/*
 * chassis_can_message_task.c
 *
 *  Created on: Oct 25, 2025
 *      Author: zhan-hao
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "chassis_can_message_task.h"
#include "gimbal_control_task.h"
#include "control_input_task.h"
#include "can_msg_processor.h"

/* Private define ------------------------------------------------------------*/
#define CAN_TX_PERIOD_MS 5 // CAN transmission period

/* Private variables ---------------------------------------------------------*/
static float lvl_max_speed;
static float lvl_max_accel;
static float lvl_max_spin;
static float spin_accel = SPIN_ACCELERATION;

float rel_forward;
float rel_horizontal;
float rel_yaw;

/* External variables --------------------------------------------------------*/
supercap_data supercap;

/* Exported variables -------------------------------------------------------*/
extern ref_game_robot_data_t ref_robot_data;

/* Private function prototypes -----------------------------------------------*/
void level_config(float *lvl_max_speed, float *lvl_max_accel, float *lvl_max_spin);
float rpm_ramp(float target_value, float current_value, float *lvl_max_accel);
int16_t pack_value(float x);

void chassis_can_message_task(void *argument) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_buffer[8];
    TickType_t xLastWakeTime;

    // Configure CAN TX header
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    //Speed and acceleration control variables
    float limit_forward;
    float limit_horizontal;
    float limit_yaw;
    float act_forward = 0.0f;
    float act_horizontal = 0.0f;
    float act_yaw = 0.0f;

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while(1) {

    	float rel_angle = yaw_motor.angle_data.adj_ang;

    	// Setting translational and rotational speed and acceleration base on robot level
    	level_config(&lvl_max_speed, &lvl_max_accel, &lvl_max_spin);

    	float speed_limit = lvl_max_speed;
    	float spin_limit = lvl_max_spin;

    	// Increase speed when spinspin mode is deactivated
    	if (chassis_ctrl_data.g_spinspin_mode == 0) {
    		speed_limit += CHASSIS_SPEED_BOOST;
    	}

    	// Clamp the values between -limit to limit
    	limit_forward = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.forward, speed_limit));
    	limit_horizontal = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.horizontal, speed_limit));
    	limit_yaw = fmaxf(-spin_limit,
    			fminf(chassis_ctrl_data.yaw, spin_limit));

    	// Smooths speed changes over time using acceleration constraints
    	act_forward = rpm_ramp(limit_forward, act_forward, &lvl_max_accel);
    	act_horizontal = rpm_ramp(limit_horizontal, act_horizontal, &lvl_max_accel);
    	act_yaw = rpm_ramp(limit_yaw, act_yaw, &spin_accel);

    	// translation and rotation speed of chassis for chassis yaw angle relative to gimbal
    	rel_forward = (act_forward * cos(rel_angle))
    			- (act_horizontal * sin(rel_angle));
    	rel_horizontal = (act_forward * sin(rel_angle))
    			+ (act_horizontal * cos(rel_angle));
    	rel_yaw = act_yaw;

    	// convert from float to int16_t
    	int16_t send_forward = pack_value(rel_forward);
    	int16_t send_horizontal = pack_value(rel_horizontal);
    	int16_t send_yaw = pack_value(rel_yaw);

    	// pack enable_supercap_module and
    	uint8_t last_byte = 0;
    	/* Bit 7 = supercap */
    	if (supercap.supercap_enabled) {
    	    last_byte |= (1 << 7);  // set MSB
    	}
    	/* Bits 6-0 = power limit (mask to 7 bits just in case) */
    	last_byte |= (ref_robot_data.chassis_power_limit & 0x7F);

    	// ===== Send CHASSIS_DATA_1 ===== //
        memset(tx_buffer, 0, 8);

        tx_buffer[0] = send_forward & 0xFF;
        tx_buffer[1] = send_forward >> 8;
        tx_buffer[2] = send_horizontal & 0xFF;
        tx_buffer[3] = send_horizontal >> 8;
        tx_buffer[4] = send_yaw & 0xFF;
        tx_buffer[5] = send_yaw >> 8;
        tx_buffer[6] = chassis_ctrl_data.enabled;           // set explicitly
        tx_buffer[7] = ref_robot_data.chassis_power_limit;  // set explicitly
        tx_header.StdId = DEV_C_TOP_TO_BOT_ID;

        // Wait for a free mailbox and send
        while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            vTaskDelay(1);  // Wait 1ms if all mailboxes are full
        }

        if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buffer, &tx_mailbox) != HAL_OK) {
            // Handle error if needed
            Error_Handler();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
    }
}

int16_t pack_value(float x) {
    return (int16_t)lroundf(x * SCALE);
}


void level_config(float *lvl_max_speed, float *lvl_max_accel,
		float *lvl_max_spin) {
#ifdef LVL_TUNING
	//	static uint8_t prev_robot_level = -1;

	//	// Hopefully with this, we can adjust pid values without it being overwritten all the time
	//	if (prev_robot_level == ref_robot_data.robot_level) return;
	//	prev_robot_level = ref_robot_data.robot_level;
	uint8_t curr_level = ref_robot_data.robot_level;

	if (supercap.supercap_enabled) {
		curr_level += 10;
	}

	switch (curr_level) {
	case 1:
		*lvl_max_speed = LV1_MAX_SPEED;
		*lvl_max_accel = LV1_MAX_ACCEL;
		*lvl_max_spin = LV1_CHASSIS_YAW_MAX_RPM;
		break;

	case 2:
		*lvl_max_speed = LV2_MAX_SPEED;
		*lvl_max_accel = LV2_MAX_ACCEL;
		*lvl_max_spin = LV2_CHASSIS_YAW_MAX_RPM;
		break;

	case 3:
		*lvl_max_speed = LV3_MAX_SPEED;
		*lvl_max_accel = LV3_MAX_ACCEL;
		*lvl_max_spin = LV3_CHASSIS_YAW_MAX_RPM;
		break;

	case 4:
		*lvl_max_speed = LV4_MAX_SPEED;
		*lvl_max_accel = LV4_MAX_ACCEL;
		*lvl_max_spin = LV4_CHASSIS_YAW_MAX_RPM;
		break;

	case 5:
		*lvl_max_speed = LV5_MAX_SPEED;
		*lvl_max_accel = LV5_MAX_ACCEL;
		*lvl_max_spin = LV5_CHASSIS_YAW_MAX_RPM;
		break;

	case 6:
		*lvl_max_speed = LV6_MAX_SPEED;
		*lvl_max_accel = LV6_MAX_ACCEL;
		*lvl_max_spin = LV6_CHASSIS_YAW_MAX_RPM;
		break;

	case 7:
		*lvl_max_speed = LV7_MAX_SPEED;
		*lvl_max_accel = LV7_MAX_ACCEL;
		*lvl_max_spin = LV7_CHASSIS_YAW_MAX_RPM;
		break;

	case 8:
		*lvl_max_speed = LV8_MAX_SPEED;
		*lvl_max_accel = LV8_MAX_ACCEL;
		*lvl_max_spin = LV8_CHASSIS_YAW_MAX_RPM;
		break;

	case 9:
		*lvl_max_speed = LV9_MAX_SPEED;
		*lvl_max_accel = LV9_MAX_ACCEL;
		*lvl_max_spin = LV9_CHASSIS_YAW_MAX_RPM;
		break;

	case 10:
		*lvl_max_speed = LV10_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 11:
		*lvl_max_speed = LV11_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 12:
		*lvl_max_speed = LV12_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 13:
		*lvl_max_speed = LV13_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 14:
		*lvl_max_speed = LV14_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 15:
		*lvl_max_speed = LV15_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 16:
		*lvl_max_speed = LV16_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 17:
		*lvl_max_speed = LV17_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 18:
		*lvl_max_speed = LV18_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 19:
		*lvl_max_speed = LV19_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 20:
		*lvl_max_speed = LV20_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	default:
		*lvl_max_speed = LV1_MAX_SPEED;
		*lvl_max_accel = LV1_MAX_ACCEL;
		*lvl_max_spin = LV1_CHASSIS_YAW_MAX_RPM;
	}
#else

	*lvl_max_speed = MAX_SPEED;
	*lvl_max_accel = MAX_ACCEL;
	*lvl_max_spin  = CHASSIS_YAW_MAX_RPM;

#endif
	*lvl_max_speed = (*lvl_max_speed < 0) ? 0 : *lvl_max_speed; //Make sure is within 0 - 1 since it is a percentage
	*lvl_max_speed = (*lvl_max_speed > 1) ? 1 : *lvl_max_speed; // Cap the max speed of motor
}

float rpm_ramp(float target_value, float current_value, float *lvl_max_accel) {
	double dt = CHASSIS_DELAY / 1000.0; // Converting dt to minutes
	double accel = *lvl_max_accel; //Default Chassis_Accel_max is LV1_ACCEL_MAX

	double ramp_rate = accel * dt; //Calc ramp_rate from max_accel
	float delta = target_value - current_value;

	if (target_value == 0) {
		return 0; //Instantly stop the robot;
	} else if (fabs(delta) < ramp_rate) {
		return target_value;  // close enough, just snap to target
	} else {
		return current_value + (delta > 0 ? ramp_rate : -ramp_rate);
	}
}

