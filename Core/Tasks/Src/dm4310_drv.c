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
* @brief:      	dm4310_enable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷�����������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
uint32_t dm_mailbox[3];
CAN_TxHeaderTypeDef dm_TxHeader;
CAN_RxHeaderTypeDef dm_RxHeader;
uint8_t RxData[8];
motor_t MF_motor[2];
motor_t motor[num];
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
float dm_set_tor[4] = {0};
float mf_set_tor[2] = {0};
extern int chassis_state;
uint8_t joint_motor_online = 0;

float dm_task_dt = 0;

void dm_motor_control_task(void *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t dm_task_lastTick = HAL_GetTick();
	dm_set_tor[0] = 0.0f;
	dm_set_tor[1] = 0.0f;
	dm_set_tor[2] = 0.0f;
	dm_set_tor[3] = 0.0f;
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
        	DWT_Delay(0.0002);
        	enableMFMotor(&hcan2, 0x141);
        	DWT_Delay(0.0002);
        	enableMFMotor(&hcan2, 0x142);
        	DWT_Delay(0.0002);
//        	dm4310_motor_init();
        }
    }
}

void dm4310_motor_init(void)
  {
  	// ��ʼ��Motor1��Motor2�ĵ���ṹ
  	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
  	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
  	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
  	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
  	memset(&MF_motor[0], 0, sizeof(MF_motor[0]));
  	memset(&MF_motor[1], 0, sizeof(MF_motor[1]));

//  	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
//  	memset(&motor[Motor6], 0, sizeof(motor[Motor6]));

  	// ����Motor1�ĵ����Ϣ
  	motor[Motor1].id = 0x81;
  	motor[Motor1].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
  	motor[Motor1].ctrl.tor_set = 0.0f;
//  	motor[Motor1].ctrl.vel_set = 3.0f;
//  	motor[Motor1].ctrl.pos_set = 0.0f;

  	motor[Motor2].id = 0x82;
  	motor[Motor2].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
  	motor[Motor2].ctrl.tor_set = 0.0f;
//  	motor[Motor2].ctrl.vel_set = 3.0f;
//  	motor[Motor2].ctrl.pos_set = 0.0f;
//
  	motor[Motor3].id = 0x83;
  	motor[Motor3].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
  	motor[Motor3].ctrl.tor_set = 0.0f;
//  	motor[Motor3].ctrl.vel_set = 3.0f;
//  	motor[Motor3].ctrl.pos_set = 0.0f;

  	motor[Motor4].id = 0x84;
  	motor[Motor4].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
  	motor[Motor4].ctrl.tor_set = 0.0f;
//  	motor[Motor4].ctrl.vel_set = 3.0f;
//  	motor[Motor4].ctrl.pos_set = 0.0f;
//
//  	motor[Motor5].id = 0x05;
//  	motor[Motor5].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
//  	motor[Motor5].ctrl.vel_set = 1.0f;
//  	motor[Motor5].ctrl.kd_set = 1.0f;
//
//  	motor[Motor6].id = 0x06;
//  	motor[Motor6].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
//  	motor[Motor6].ctrl.vel_set = 1.0f;
//  	motor[Motor6].ctrl.kd_set = 1.0f;
  	MF_motor[0].ctrl.tor_set = 0.0f;
  	MF_motor[1].ctrl.tor_set = 0.0f;
  	dm4310_enable(&hcan2, &motor[Motor1]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor2]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor3]);
  	vTaskDelay(1);
  	dm4310_enable(&hcan2, &motor[Motor4]);
  	vTaskDelay(1);
  	enableMFMotor(&hcan2, 0x141);
  	enableMFMotor(&hcan2, 0x142);
  	MF_motor[0].initialized = 1;
  	MF_motor[1].initialized = 1;

//  	save_pos_zero(&hcan2, 0x81, 0);
//  	vTaskDelay(1);
//  	save_pos_zero(&hcan2, 0x82, 0);
//  	vTaskDelay(1);
//  	save_pos_zero(&hcan2, 0x83, 0);
//  	vTaskDelay(1);
//  	save_pos_zero(&hcan2, 0x84, 0);
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

void enableMFMotor(CAN_HandleTypeDef* hcan,int id) {
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

void disableMFMotor(CAN_HandleTypeDef* hcan,int id) {
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
* @brief:      	dm4310_disable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷��ͽ�������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
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
* @brief:      	dm4310_ctrl_send: ����DM4310������������
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ�����DM4310���
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
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
* @brief:      	dm4310_set: ����DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���������������DM4310����Ŀ��Ʋ���������λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
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
* @brief:      	dm4310_clear: ���DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	��DM4310�������������Ϳ��Ʋ������㣬����λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
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
* @brief:      	dm4310_clear_err: ���DM4310���������
* @param[in]:   hcan: 	 ָ��CAN���ƽṹ���ָ��
* @param[in]:  	motor:   ָ�����ṹ���ָ��
* @retval:     	void
* @details:    	���ݵ���Ŀ���ģʽ�����ö�Ӧģʽ�����������
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
* @brief:      	dm4310_fbdata: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť���Լ�����¶Ȳ���
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
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
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
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
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
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
* @brief:      	enable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
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
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
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
* @brief:      	save_pos_zero: ����λ����㺯��
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ����λ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͱ���λ����������
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
* @brief:      	clear_err: ������������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ��������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������������
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
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
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
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
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
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel)
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
void MFspeed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
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
* @brief:      	pos_speed_ctrl: ���ģʽ
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   i:				��������ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void pos_force_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, uint16_t vel, uint16_t i)
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
