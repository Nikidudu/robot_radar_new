/**
************************************************************************
* @file:        dm4310_drv.c
* @brief:       Driver for DM4310 motor control and MF motor control
* @details:     This file implements the control logic for DM4310 motors and MF motors,
*               including initialization, communication, and feedback handling.
************************************************************************
**/

#include "dm4310_drv.h"
#include <string.h>
#include "board_lib.h"
#include "robot_config.h"
#include "can_msg_processor.h"
#include "motor_control.h"
#include "motor_control_task.h"
#include "motor_config.h"

/**
************************************************************************
* @brief:      	dm4310_enable: DM4310ģʽ
* @param[in]:   hcan:    ָCAN_HandleTypeDefṹָ
* @param[in]:   motor:   ָmotor_tṹָ룬ϢͿƲ
* @retval:     	void
* @details:    	ݵģʽӦģʽͨCAN߷
*               ֵ֧Ŀģʽλģʽλٶȿģʽٶȿģʽ
************************************************************************
**/
// CAN communication variables
uint32_t dm_mailbox[3];
CAN_TxHeaderTypeDef dm_TxHeader;

// Motor structures
motor_t MF_motor[2];
motor_t motor[num];
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;

// Torque command arrays
float dm_set_tor[4] = {0};
float mf_set_tor[2] = {0};
float pitch_set_tor = 0;

extern int chassis_state;
uint8_t joint_motor_online = 0;

float dm_task_dt = 0;

/**
************************************************************************
* @brief:      Motor control task function
* @param[in]:  argument - Task argument (unused)
* @retval:     void
* @details:    Main control loop for motor control:
*              1. Initializes motors and parameters
*              2. Updates motor commands and feedback
*              3. Monitors motor status and handles errors
*              4. Maintains communication with motors
************************************************************************
**/
void dm_motor_control_task(void *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t dm_task_lastTick = HAL_GetTick();
	dm_set_tor[0] = 0.0f;
	dm_set_tor[1] = 0.0f;
	dm_set_tor[2] = 0.0f;
	dm_set_tor[3] = 0.0f;
	pitch_set_tor = 0.0f;
	osDelay(1000);
	dm4310_motor_init();
	osDelay(100);

    while (1) {
    	uint32_t currentTick = HAL_GetTick();
    	// Calculate dt in seconds (since HAL_GetTick returns milliseconds)
    	dm_task_dt = (currentTick - dm_task_lastTick) / 1000.0f;
    	dm_task_lastTick = currentTick; // update for next call
//    	motor[Motor1].ctrl.tor_set = 0.1f;
//    	motor[Motor2].ctrl.tor_set = 0.2f;
//    	motor[Motor3].ctrl.tor_set = -0.1f;
//    	motor[Motor4].ctrl.tor_set = -0.2f;
    	motor[Motor1].ctrl.tor_set = dm_set_tor[0];
    	motor[Motor2].ctrl.tor_set = dm_set_tor[1];
    	motor[Motor3].ctrl.tor_set = dm_set_tor[2];
    	motor[Motor4].ctrl.tor_set = dm_set_tor[3];
    	motor[Motor5].ctrl.tor_set = pitch_set_tor;
    	MF_motor[0].ctrl.tor_set = mf_set_tor[0];
    	MF_motor[1].ctrl.tor_set = mf_set_tor[1];
    	leftJoint[0].angle = motor[Motor4].para.pos;
    	leftJoint[1].angle = motor[Motor1].para.pos;
    	rightJoint[0].angle = motor[Motor2].para.pos;
    	rightJoint[1].angle = motor[Motor3].para.pos;
    	leftJoint[0].speed = motor[Motor4].para.vel;
    	leftJoint[1].speed = motor[Motor1].para.vel;
    	rightJoint[0].speed = motor[Motor2].para.vel;
    	rightJoint[1].speed = motor[Motor3].para.vel;
    	leftWheel.angle = MF_motor[0].para.encoder_angle;
    	rightWheel.angle = MF_motor[1].para.encoder_angle;
    	leftWheel.speed = MF_motor[0].para.speed;
    	rightWheel.speed = MF_motor[1].para.speed;
    	leftWheel.torque = MF_motor[0].para.torque;
    	rightWheel.torque = MF_motor[1].para.torque;

    	dm4310_ctrl_send(&hcan2, &motor[Motor1]);
    	motor[Motor1].para.ping += 1;
    	dm4310_ctrl_send(&hcan2, &motor[Motor2]);
    	motor[Motor2].para.ping += 1;
    	DWT_Delay(0.0005);
    	dm4310_ctrl_send(&hcan2, &motor[Motor3]);
    	motor[Motor3].para.ping += 1;
    	dm4310_ctrl_send(&hcan2, &motor[Motor4]);
    	motor[Motor4].para.ping += 1;

    	dm4310_ctrl_send(&hcan1, &motor[Motor5]);
    	motor[Motor5].para.ping += 1;
    	osDelay(1);

        MFtorque_command(&hcan2, 0x141, MF_motor[0].ctrl.tor_set);
        MF_motor[0].para.ping += 1;
//        DWT_Delay(0.0002);
        MFtorque_command(&hcan2, 0x142, -MF_motor[1].ctrl.tor_set);
        MF_motor[1].para.ping += 1;
//        DWT_Delay(0.0002);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3));
        if (motor[Motor1].para.ping > 50 || motor[Motor1].para.state != 9){
        	motor[Motor1].para.online = 0;
        }else{
        	motor[Motor1].para.online = 1;
        }
        if (motor[Motor2].para.ping > 50 || motor[Motor2].para.state != 9){
        	motor[Motor2].para.online = 0;
        }else{
        	motor[Motor2].para.online = 1;
        }
        if (motor[Motor3].para.ping > 50 || motor[Motor3].para.state != 9){
        	motor[Motor3].para.online = 0;
        }else{
        	motor[Motor3].para.online = 1;
        }
        if (motor[Motor4].para.ping > 50 || motor[Motor4].para.state != 9){
        	motor[Motor4].para.online = 0;
        }else{
        	motor[Motor4].para.online = 1;
        }
        if (motor[Motor5].para.ping > 50 || motor[Motor5].para.state != 9){
        	motor[Motor5].para.online = 0;
        }else{
        	motor[Motor5].para.online = 1;
        }
        if (MF_motor[0].para.ping > 50){
        	MF_motor[0].para.online = 0;
        }else{
        	MF_motor[0].para.online = 1;
        }
        if (MF_motor[1].para.ping > 50){
        	MF_motor[1].para.online = 0;
        }else{
        	MF_motor[1].para.online = 1;
        }
        if (motor[Motor1].para.online == 1 &&
        		motor[Motor2].para.online == 1 &&
				motor[Motor3].para.online == 1 &&
				MF_motor[0].para.online == 1 &&
				MF_motor[1].para.online == 1 &&
				motor[Motor4].para.online == 1){
        	joint_motor_online = 1;
        }else{
        	joint_motor_online = 0;
//        	HAL_CAN_Stop(&hcan2);
//        	osDelay(5);  // Wait for motor power stabilization
//        	HAL_CAN_Start(&hcan2);
        	osDelay(1);
        	dm4310_enable(&hcan2, &motor[Motor1]);
        	dm4310_enable(&hcan2, &motor[Motor2]);
        	DWT_Delay(0.0002);
        	dm4310_enable(&hcan2, &motor[Motor3]);
        	dm4310_enable(&hcan2, &motor[Motor4]);
        	dm4310_enable(&hcan1, &motor[Motor5]);
        	DWT_Delay(0.0002);
//        	dm4310_motor_init();
        }
    }
}

/**
************************************************************************
* @brief:      Motor initialization function
* @param[in]:  None
* @retval:     void
* @details:    Initializes all motors (DM4310 and MF) with default parameters:
*              1. Clears all motor structures
*              2. Sets motor IDs and control modes
*              3. Initializes torque settings
*              4. Enables motors in appropriate modes
************************************************************************
**/
void dm4310_motor_init(void)
{
  	// Initialize motor structures with zeros
  	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
  	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
  	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
  	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
  	memset(&MF_motor[0], 0, sizeof(MF_motor[0]));
  	memset(&MF_motor[1], 0, sizeof(MF_motor[1]));
  	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));

  	// Configure Motor1 (Left Joint 1)
  	motor[Motor1].id = 0x81;
  	motor[Motor1].ctrl.mode = 0;		// MIT mode for precise torque control
  	motor[Motor1].ctrl.tor_set = 0.0f;

  	// Configure Motor2 (Right Joint 1)
  	motor[Motor2].id = 0x82;
  	motor[Motor2].ctrl.mode = 0;		// MIT mode for precise torque control
  	motor[Motor2].ctrl.tor_set = 0.0f;

  	// Configure Motor3 (Right Joint 2)
  	motor[Motor3].id = 0x83;
  	motor[Motor3].ctrl.mode = 0;		// MIT mode for precise torque control
  	motor[Motor3].ctrl.tor_set = 0.0f;

  	// Configure Motor4 (Left Joint 2)
  	motor[Motor4].id = 0x84;
  	motor[Motor4].ctrl.mode = 0;		// MIT mode for precise torque control
  	motor[Motor4].ctrl.tor_set = 0.0f;

  	// Configure Motor5 (Pitch Motor)
  	motor[Motor5].id = 0x85;
  	motor[Motor5].ctrl.mode = 0;		// MIT mode for precise torque control
  	motor[Motor5].ctrl.tor_set = 0.0f;

  	// Initialize MF motors (Wheels)
  	MF_motor[0].ctrl.tor_set = 0.0f;  // Left wheel
  	MF_motor[1].ctrl.tor_set = 0.0f;  // Right wheel

  	// Enable all motors with appropriate delays
  	dm4310_enable(&hcan2, &motor[Motor1]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor2]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor3]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor4]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan1, &motor[Motor5]);
  	vTaskDelay(1);
  	enableMFMotor(&hcan2, 0x141);  // Enable left wheel
  	enableMFMotor(&hcan2, 0x142);  // Enable right wheel

  	// Mark MF motors as initialized
  	MF_motor[0].initialized = 1;
  	MF_motor[1].initialized = 1;
}

// Callback function to handle CAN receive interrupt
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &dm_RxHeader, RxData);
//    MF_fbdata(&MF_motor, &RxData[0]);
////    fb_id = (RxData[0])&0x0F;
////    switch(fb_id)
////    	{
////    		case 1:
////    			dm4310_fbdata(&motor[Motor1],&RxData[0]);
////    			break;
////    		case 2:
////    			dm4310_fbdata(&motor[Motor2],&RxData[0]);
////    			break;
////    		case 3:
////    			dm4310_fbdata(&motor[Motor3],&RxData[0]);
////    			break;
////    		case 4:
////    			dm4310_fbdata(&motor[Motor4],&RxData[0]);
////    			break;
////    	}
//}

/**
************************************************************************
* @brief:      Enable MF motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  id - CAN ID of the motor to enable
* @retval:     void
* @details:    Sends enable command to MF motor via CAN bus
************************************************************************
**/
void enableMFMotor(CAN_HandleTypeDef* hcan, int id) {
	uint8_t data[8];
	dm_TxHeader.DLC = 0x08;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = id;
	//id.StdId = motor_id + mode_id;
	data[0] = 0x88;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Disable MF motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  id - CAN ID of the motor to disable
* @retval:     void
* @details:    Sends disable command to MF motor via CAN bus
************************************************************************
**/
void disableMFMotor(CAN_HandleTypeDef* hcan, int id) {
	uint8_t data[8];
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = id;
	//id.StdId = motor_id + mode_id;
	data[0] = 0x80;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Enable DM4310 motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor - Pointer to motor structure containing configuration
* @retval:     void
* @details:    Enables DM4310 motor in specified mode:
*              - MIT mode (0): Precise torque control
*              - Position mode (1): Position control
*              - Speed mode (2): Velocity control
*              - Position-force mode (3): Combined position and force control
************************************************************************
**/
void dm4310_enable(CAN_HandleTypeDef* hcan, motor_t* motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
		case 3:
			enable_motor_mode(hcan, motor->id, POSI_MODE);
			break;
	}	
}

/**
************************************************************************
* @brief:      	dm4310_disable: DM4310ģʽ
* @param[in]:   hcan:    ָCAN_HandleTypeDefṹָ
* @param[in]:   motor:   ָmotor_tṹָ룬ϢͿƲ
* @retval:     	void
* @details:    	ݵ����ģʽӦģʽͨCAN߷ͽ
*               ֵ֧Ŀģʽλģʽλٶȿģʽٶȿģʽ
************************************************************************
**/
void dm4310_disable(CAN_HandleTypeDef* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
		case 3:
			disable_motor_mode(hcan, motor->id, POSI_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}

/**
************************************************************************
* @brief:      Send control command to DM4310 motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor - Pointer to motor structure containing control parameters
* @retval:     void
* @details:    Sends control commands based on motor mode:
*              - MIT mode: Position, velocity, KP, KD, and torque
*              - Position mode: Position and velocity
*              - Speed mode: Velocity only
*              - Position-force mode: Position, velocity, and force
************************************************************************
**/
void dm4310_ctrl_send(CAN_HandleTypeDef* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case 3:
			pos_force_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.tor_set);
			break;
	}	
}

/**
************************************************************************
* @brief:      Set DM4310 motor control parameters
* @param[in]:  motor - Pointer to motor structure
* @retval:     void
* @details:    Updates motor control parameters from command structure:
*              - KD (derivative gain)
*              - KP (proportional gain)
*              - Position setpoint
*              - Velocity setpoint
*              - Torque setpoint
************************************************************************
**/
void dm4310_set(motor_t *motor)
{
	motor->ctrl.kd_set 	= motor->cmd.kd_set;
	motor->ctrl.kp_set	= motor->cmd.kp_set;
	motor->ctrl.pos_set	= motor->cmd.pos_set;
	motor->ctrl.vel_set	= motor->cmd.vel_set;
	motor->ctrl.tor_set	= motor->cmd.tor_set;

}

/**
************************************************************************
* @brief:      Clear DM4310 motor parameters
* @param[in]:  motor - Pointer to motor structure
* @retval:     void
* @details:    Resets all motor control parameters to zero:
*              - Command parameters (KD, KP, position, velocity, torque)
*              - Control parameters (KD, KP, position, velocity, torque)
************************************************************************
**/
void dm4310_clear_para(motor_t *motor)
{
	motor->cmd.kd_set 	= 0;
	motor->cmd.kp_set	 	= 0;
	motor->cmd.pos_set 	= 0;
	motor->cmd.vel_set 	= 0;
	motor->cmd.tor_set 	= 0;
	
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}

/**
************************************************************************
* @brief:      	dm4310_clear_err: DM4310
* @param[in]:   hcan: 	 ָCANƽṹָ
* @param[in]:  	motor:   ָ�����ṹ���ָ��
* @retval:     	void
* @details:    	���ݵ���ĿģʽöӦģʽ
************************************************************************
**/
void dm4310_clear_err(CAN_HandleTypeDef* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

/**
************************************************************************
* @brief:      Process DM4310 motor feedback data
* @param[in]:  motor - Pointer to motor structure to store feedback
* @param[in]:  rx_data - Pointer to received CAN data
* @retval:     void
* @details:    Processes and stores motor feedback data:
*              - Motor ID and state
*              - Position, velocity, and torque
*              - Temperature (MOSFET and coil)
*              Applies necessary scaling and sign corrections
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-3.14,3.14)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
	if (motor->para.id == 2 || motor->para.id == 3){
		motor->para.pos *= -1.0f ;
		motor->para.vel *= -1.0f ;
	}
	if (motor->para.id == 2 || motor->para.id == 4){
		motor->para.pos += 3.142f;
	}
}

void MF_fbdata(motor_t *motor, uint8_t *rx_data, uint32_t id)
{
    // Parse motor temperature directly from DATA[1]
    motor->para.temperature = (int8_t)rx_data[1];

    // Parse torque current (iq) from DATA[2] and DATA[3] as a 16-bit signed integer
    int16_t iq_raw = (int16_t)((rx_data[2]) | (rx_data[3] << 8));
    motor->para.torque = iq_raw * (5.28f / 2048.0f);

    // Parse motor speed from DATA[4] and DATA[5] as a 16-bit signed integer
    motor->para.speed = ((int16_t)((rx_data[4]) | (rx_data[5] << 8))) * (73.303f / 2820.0f);

    // Parse encoder position from DATA[6] and DATA[7] as a 16-bit unsigned integer
    uint16_t encoder_raw = (uint16_t)((rx_data[6]) | (rx_data[7] << 8));

//    if (motor->initialized == 1 && encoder_raw != 0){
//    	motor->initial_angle_offset = encoder_raw;
//    	motor->initialized = 0;
//    }
//    encoder_raw = encoder_raw - motor->initial_angle_offset;
    // Convert raw encoder value to radians
    float current_angle = encoder_raw * (2.0f * M_PI / 65535.0f); // Map 0 to 65535 -> 0 to 2π radians

    // Calculate the angle difference to detect wrapping
    float delta_angle = current_angle - motor->para.previous_angle;

    // Handle positive and negative wrapping
    if (delta_angle > M_PI) {
        motor->para.rotations--; // Crossed the 0 -> 2π boundary
    } else if (delta_angle < -M_PI) {
        motor->para.rotations++; // Crossed the 2π -> 0 boundary
    }

    // Calculate the continuous angle in radians
    motor->para.encoder_angle = current_angle + (motor->para.rotations * 2.0f * M_PI);

    // Update the previous angle for the next iteration
    motor->para.previous_angle = current_angle;
    if (id == 0x142){
    	motor->para.encoder_angle = -motor->para.encoder_angle;
    	motor->para.speed = -motor->para.speed;
    	motor->para.torque = -motor->para.torque;
    }
}

/**
************************************************************************
* @brief:      Convert float to unsigned integer
* @param[in]:  x_float - Float value to convert
* @param[in]:  x_min - Minimum value of range
* @param[in]:  x_max - Maximum value of range
* @param[in]:  bits - Number of bits for result
* @retval:     Unsigned integer result
* @details:    Maps float value from [x_min, x_max] to [0, 2^bits-1]
*              Used for converting control values to CAN protocol format
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      Convert unsigned integer to float
* @param[in]:  x_int - Integer value to convert
* @param[in]:  x_min - Minimum value of range
* @param[in]:  x_max - Maximum value of range
* @param[in]:  bits - Number of bits in input
* @retval:     Float result
* @details:    Maps unsigned integer from [0, 2^bits-1] to [x_min, x_max]
*              Used for converting CAN protocol values to control values
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      Enable motor mode
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  mode_id - Operating mode to enable
* @retval:     void
* @details:    Sends enable command for specified motor mode:
*              - Sets up CAN frame with enable command
*              - Handles mailbox overflow conditions
*              - Ensures reliable transmission
************************************************************************
**/
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = motor_id + mode_id;
	//id.StdId = motor_id + mode_id;
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
	{
		// Abort any pending messages to free the mailbox
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
	}
	HAL_StatusTypeDef status = HAL_ERROR;
	if (status == HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox))
		status = HAL_ERROR; // HIHIHIH
}

/**
************************************************************************
* @brief:      Disable motor mode
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  mode_id - Operating mode to disable
* @retval:     void
* @details:    Sends disable command for specified motor mode:
*              - Sets up CAN frame with disable command
*              - Ensures proper mode deactivation
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Save zero position for motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  mode_id - Operating mode for position save
* @retval:     void
* @details:    Saves current position as zero reference:
*              - Used for calibrating motor position
*              - Important for position control modes
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Clear motor error state
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  mode_id - Operating mode for error clear
* @retval:     void
* @details:    Clears error state of motor:
*              - Resets error flags
*              - Allows motor to resume operation
************************************************************************
**/
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      MIT mode control command
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  pos - Position setpoint
* @param[in]:  vel - Velocity setpoint
* @param[in]:  kp - Position gain
* @param[in]:  kd - Velocity gain
* @param[in]:  torq - Torque setpoint
* @retval:     void
* @details:    Sends MIT mode control command:
*              - Converts float parameters to protocol format
*              - Packs data into CAN frame
*              - Handles all MIT mode control parameters
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	dm_TxHeader.StdId = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Position and speed control command
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  pos - Position setpoint
* @param[in]:  vel - Velocity setpoint
* @retval:     void
* @details:    Sends position-speed control command:
*              - Packs position and velocity into CAN frame
*              - Used for combined position-velocity control
************************************************************************
**/
void pos_speed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel)
{
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR

	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	dm_TxHeader.StdId = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Speed control command
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  vel - Velocity setpoint
* @retval:     void
* @details:    Sends speed control command:
*              - Packs velocity setpoint into CAN frame
*              - Used for pure velocity control mode
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	uint8_t *vbuf;
	uint8_t data[4];
	
	dm_TxHeader.StdId = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

/**
************************************************************************
* @brief:      Send speed control command to MF motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  vel - Desired velocity in degrees per second
* @retval:     void
* @details:    Sends speed control command to MF motor:
*              - Command byte 0xA2 for speed control
*              - Speed value scaled by 100 for protocol format
*              - Handles error conditions during transmission
************************************************************************
**/
void MFspeed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel) {
    // Setting up the CAN header for 8 bytes data and standard frame
    dm_TxHeader.DLC = 8;                   // Data Length Code: 8 bytes of data
    dm_TxHeader.IDE = CAN_ID_STD;          // Standard CAN ID
    dm_TxHeader.RTR = CAN_RTR_DATA;        // Remote Transmission Request (Data frame)

    // Set the command ID by adding the motor ID to the base ID
    dm_TxHeader.StdId = motor_id;

    // Prepare the data array, with command byte for speed control
    uint8_t data[8] = {0};                 // Initialize all bytes to 0
    data[0] = 0xA2;              // Command byte for speed control

    // Convert the velocity from float to int32_t (scaled for protocol)
    int32_t speedControl = (int32_t)(vel * 100);  // Assuming vel is in dps, multiply to match 0.01 dps/LSB

    // Fill in the data bytes with speedControl value
    data[4] = (speedControl >> 0) & 0xFF;
    data[5] = (speedControl >> 8) & 0xFF;
    data[6] = (speedControl >> 16) & 0xFF;
    data[7] = (speedControl >> 24) & 0xFF;

    // Send the CAN message
    uint32_t dm_mailbox[3];
    if (HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox) != HAL_OK) {
        // Handle transmission error
        Error_Handler();
    }
}

/**
************************************************************************
* @brief:      Send torque control command to MF motor
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  desired_torque - Desired torque in Nm
* @retval:     void
* @details:    Sends torque control command to MF motor:
*              - Command byte 0xA1 for torque control
*              - Converts torque to current units for motor
*              - Handles transmission error conditions
************************************************************************
**/
void MFtorque_command(CAN_HandleTypeDef* hcan, uint16_t motor_id, float desired_torque)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t data[8] = {0}; // Initialize data array to 0

    // Set up the CAN header
    txHeader.DLC = 8;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = motor_id;

    // Set command byte for torque control
    data[0] = 0xA1;

    // Convert desired torque (A) to iqControl value
    int16_t iqControl = (int16_t)(desired_torque * (2048.0f / 5.28f));

    // Set the iqControl value into data[4] and data[5]
    data[4] = (uint8_t)(iqControl & 0xFF);        // Low byte
    data[5] = (uint8_t)((iqControl >> 8) & 0xFF); // High byte

    // Send the CAN message
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(hcan, &txHeader, data, &mailbox) != HAL_OK) {
        // Transmission error handling
//        Error_Handler();
    }
}

/**
************************************************************************
* @brief:      Position and force control command
* @param[in]:  hcan - Pointer to CAN handle structure
* @param[in]:  motor_id - CAN ID of target motor
* @param[in]:  pos - Position setpoint
* @param[in]:  vel - Velocity limit
* @param[in]:  i - Force/current setpoint
* @retval:     void
* @details:    Sends position-force control command:
*              - Combines position control with force limiting
*              - Used for force-sensitive positioning
************************************************************************
**/
void pos_force_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, uint16_t vel, uint16_t i)
{
	dm_TxHeader.DLC = 8;  //The Data Bytes of the data. For the C620, it is 8 bytes of data.
	dm_TxHeader.IDE = CAN_ID_STD;  //Standard CAN BUS transmission
	dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	dm_TxHeader.StdId = motor_id + POSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	ibuf=(uint8_t*)&i;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}
