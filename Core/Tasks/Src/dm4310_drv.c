#include "dm4310_drv.h"
#include <string.h>
#include "board_lib.h"
#include "robot_config.h"
#include "can_msg_processor.h"
#include "motor_control.h"
#include "motor_control_task.h"
#include "motor_config.h"
#include "PID.h"
#include "INS_task.h"


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
motor_t dm_pitch_motor;
motor_t dm_yaw_motor;

float dm_set_tor[2];
extern INS_t INS;
extern gimbal_control_t gimbal_ctrl_data;
extern remote_cmd_t g_remote_cmd;
extern orientation_data_t imu_heading;

extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;
extern chassis_control_t chassis_ctrl_data;

uint8_t joint_motor_online = 0;
extern uint8_t g_safety_toggle;

float test = 0;

CascadePID gimbal_cpid_pit;
CascadePID gimbal_cpid_yaw;
PID gimbal_pid_yaw;
PID gimbal_pid_pitch;
float yaw_error = 0;
float target_rad = 0;
float target_gimbal = 0.0f; //gimbal center
float dumbasss;
float debug1 = 1;
float debug2 = 0;
float test3 = 0;
float dm1 = 10;
float dm2 = 5;
float ex_pos = 0.0;
float debug3 = 0.0;

void dm_motor_control_task(void *argument) {
	dm_set_tor[0] = 0.0f;
	dm_set_tor[1] = 0.0f;

	dm4310_motor_init();
	vTaskDelay(101);

	PID_Init(&gimbal_cpid_yaw.inner, 0.3, 0, 0.1, 0, 7);
	PID_Init(&gimbal_cpid_yaw.outer, 25, 0, 0.1, 0, 10);
	PID_Init(&gimbal_pid_yaw, 5, 0, 0, 0, 45);
	PID_Init(&gimbal_pid_pitch, 2.0, 0.0, 100.0, 0, 5);

//	float las_angle = 0.0f;
//	float angular_speed = 0.0f;
	float dt = 0.003;
	TickType_t lastTick = xTaskGetTickCount();

	float prev_yaw = imu_heading.yaw;
	ex_pos = dm_yaw_motor.para.pos;

	while (1) {
	    xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore,portMAX_DELAY);

		if(gimbal_ctrl_data.enabled == 1){
		    TickType_t currentTick = xTaskGetTickCount();
		    dt = (currentTick - lastTick) / 1000.0f;  // Convert to seconds
		    lastTick = currentTick;  // Update last tick

		    target_rad = gimbal_ctrl_data.pitch;

			if (target_rad > 0.13f){
				target_rad = 0.13f;
			    gimbal_ctrl_data.pitch = 0.13f;
			} else if(target_rad < -0.70f){
			    target_rad = -0.70f;
			    gimbal_ctrl_data.pitch = -0.70f;
			}

			PID_SingleCalc(&gimbal_pid_pitch, target_rad , INS.Pitch);


		    //float turn_ang = imu_heading.yaw - prev_yaw;
			float raw_pos = dm_yaw_motor.para.pos;
//		    float turn_ang = imu_heading.yaw - prev_yaw;
			float turn_ang = raw_pos - prev_yaw;
			gimbal_ctrl_data.delta_yaw -= turn_ang;

		    while (turn_ang > PI) {
		    	turn_ang -= 2 * PI;
		    }

		    while (turn_ang < -PI) {
		    	turn_ang += 2 * PI;
		    }

		    dumbasss = turn_ang;

		  //  gimbal_ctrl_data.delta_yaw -= turn_ang;
		    prev_yaw = raw_pos;
//		    ex_pos = dm_yaw_motor.para.pos;
//		    debug3 = dm_yaw_motor.para.pos + gimbal_ctrl_data.delta_yaw;

		    // Clamp delta_yaw to one round
//		    while (gimbal_ctrl_data.delta_yaw > 4*PI) {
//		    	gimbal_ctrl_data.delta_yaw = 4*PI;
//		    }
//		    while (gimbal_ctrl_data.delta_yaw < -4*PI) {
//		    	gimbal_ctrl_data.delta_yaw = -4*PI;
//		    }
		    gimbal_ctrl_data.delta_yaw -= turn_ang;

//		    while (debug3 > 4 * PI) {
//		    	debug3 -= 8 * PI;
//		    }
//		    while (debug3 <  4 * -PI) {
//		    	debug3 += 8 * PI;
//		    }

//		    void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data, uint8_t loopback) {
//			yangle_pid(dm_pitch_motor.para.yaw, dm_pitch_motor.para.yaw, yaw_motor,
//						imu_heading.yaw, &prev_yaw,0);

		    PID_SingleCalc(&gimbal_pid_yaw, 0, -gimbal_ctrl_data.delta_yaw);
	//	    PID_CascadeCalc(&gimbal_cpid_yaw, 0, -gimbal_ctrl_data.delta_yaw, g_can_motors[YAW_MOTOR_ID - 1].raw_data.torque);
	//	    target_rad += g_remote_cmd.right_y * 0.00001;

		    // if ((dm_pitch_motor.para.heartbeat == 0 || dm_pitch_motor.para.state != 9) && dm_pitch_motor.para.disconnect_time > 100) {
		    if (dm_pitch_motor.para.state != 9 && dm_pitch_motor.para.disconnect_time > 100) {
		    	dm_pitch_motor.para.disconnect_time = 0;
		    	dm_pitch_motor.para.online = 0;
		    }
	//	    else if ((dm_pitch_motor.para.heartbeat == 0 || dm_pitch_motor.para.state != 9)) {
	//	    	dm_pitch_motor.para.disconnect_time++;
	//	    }
		    else {
		    	dm_pitch_motor.para.disconnect_time = 0;
		    	dm_pitch_motor.para.online = 1;
		    }

		    if (dm_pitch_motor.para.online == 1) {
		        joint_motor_online = 1;
		    } else {
		        joint_motor_online = 0;
	//	        HAL_CAN_Stop(&hcan1);
	//	        osDelay(10);  // Wait for motor power stabilization
	//	        HAL_CAN_Start(&hcan1);
	//	        osDelay(10);
	//	        dm4310_enable(&hcan1, &dm_pitch_motor);
	//	        vTaskDelay(1);
		    }


		   // if ((dm_yaw_motor.para.heartbeat == 0 || dm_yaw_motor.para.state != 9) && dm_yaw_motor.para.disconnect_time > 100) {
		   if (dm_yaw_motor.para.state != 9 && dm_yaw_motor.para.disconnect_time > 100) {
			   dm_yaw_motor.para.disconnect_time = 0;
			   dm_yaw_motor.para.online = 0;
		   }
	//	   else if ((dm_yaw_motor.para.heartbeat == 0 || dm_yaw_motor.para.state != 9)) {
	//		   dm_yaw_motor.para.disconnect_time++;
	//	   }
		   else {
			   dm_yaw_motor.para.disconnect_time = 0;
			   dm_yaw_motor.para.online = 1;
		   }
		   if (dm_yaw_motor.para.online == 1) {
			   joint_motor_online = 1;
		   } else {
			   joint_motor_online = 0;
	//		   HAL_CAN_Stop(&hcan2);
	//		   osDelay(10);  // Wait for motor power stabilization
	//		   HAL_CAN_Start(&hcan2);
	//		   osDelay(10);
	//		   dm4310_enable(&hcan2, &dm_yaw_motor);
	//		   vTaskDelay(1);
		   }
	//	    dm_yaw_motor.para.heartbeat = 0;

		    // Disable pitch if kill switch is on
		    if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		    	dm_set_tor[0] = 0;
		    	dm_set_tor[1] = 0;
		    } else {
		    	dm_set_tor[0] = 0.4842f*imu_heading.pit - 2.3124f - gimbal_pid_pitch.output;
		    	dm_set_tor[0] *= 1.1 ;
//		    	dm_set_tor[1] = gimbal_pid_yaw.output * debug1  + chassis_ctrl_data.yaw * debug2;
//		    	test3 = gimbal_pid_yaw.output;
		    }

	    	dm_pitch_motor.ctrl.tor_set = dm_set_tor[0];
//			dm_yaw_motor.ctrl.tor_set = dm_set_tor[1];

			dm_yaw_motor.ctrl.vel_set = gimbal_pid_yaw.output;
			dm_yaw_motor.ctrl.tor_set = 0.0;//3.0;
			dm_yaw_motor.ctrl.pos_set = 0;
			dm_yaw_motor.ctrl.kp_set = 0;
			dm_yaw_motor.ctrl.kd_set = 2;


		} else {
			dm4310_clear_para(&dm_yaw_motor);
			dm4310_clear_para(&dm_pitch_motor);
		}


		dm4310_ctrl_send(&hcan1, &dm_pitch_motor);
		dm4310_ctrl_send(&hcan2, &dm_yaw_motor);

	    xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);

		vTaskDelay(4);

	}
}

float shortest_angular_difference(float current, float target) {
    float diff = fmodf(target - current + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    diff -= M_PI;
    return diff;
}

void dm4310_motor_init(void)
  {

#if PITCH_MOTOR_TYPE == TYPE_DM4310
  	memset(&dm_pitch_motor, 0, sizeof(dm_pitch_motor));
  	dm_pitch_motor.id = 0x81;
  	dm_pitch_motor.ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
 // dm_pitch_motor.ctrl.tor_set = -5.0f;
 // dm_pitch_motor.ctrl.vel_set = 1.0f;
  //	dm_pitch_motor.ctrl.pos_set = 0.0f;
//  	motor[Motor1].ctrl.kd_set = 0;//1.372f;
 //	dm_pitch_motor.ctrl.kp_set = 54;//54;

  	dm4310_enable(&hcan1, &dm_pitch_motor);

  	vTaskDelay(3);
#endif

#if YAW_MOTOR_TYPE == TYPE_DM4310
  	memset(&dm_yaw_motor, 0, sizeof(dm_yaw_motor));
  	dm_yaw_motor.id = 0x61;
  	dm_yaw_motor.ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
  //	dm_yaw_motor.ctrl.tor_set = 0.0f;
//  	motor[Motor2].ctrl.pos_set = 0.0f;
 // 	motor[Motor2].ctrl.vel_set = 0.0f;
  	dm4310_enable(&hcan2, &dm_yaw_motor);
  	vTaskDelay(3);
//    motor[Motor2].ctrl.vel_set = 3.0f;
//
#endif



//	save_pos_zero(&hcan1,0x81, 0);
//  	save_pos_zero(&hcan2, 0x82, 0);
//  	save_pos_zero(&hcan2, 0x83, 0);
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

void dmmapyawfbdata(motor_t *yaw_motor){
//	yaw_motor->para.pos = dm_yaw_encoder_mod(yaw_motor->para.pos);
	g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang = dm_yaw_encoder_mod(yaw_motor->para.pos);
	g_can_motors[YAW_MOTOR_ID - 1].raw_data.torque = yaw_motor->para.tor;
}

void dmmappitchfbdata(motor_t *pitch_motor){
	g_pitch_motor.angle_data.adj_ang = pitch_motor->para.pos;
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
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set,motor->ctrl.vel_set,motor->ctrl.kp_set,motor->ctrl.kd_set, motor->ctrl.tor_set);
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
	motor->cmd.kp_set	= 0;
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

	if (motor->id == 0x61){
		dmmapyawfbdata(&dm_yaw_motor);
	} else if (motor->id == 0x81){
		dmmappitchfbdata(&dm_pitch_motor);
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

float dm_yaw_encoder_mod(float raw_angle) {
	float mapped_angle = fmod(P_MAX + raw_angle, 2*P_MAX/P_ROUNDS);
	return mapped_angle - P_MAX/P_ROUNDS;
}
