#include "bsp_damiao.h"
#include <string.h>
#include "math.h"
#include "stm32f4xx_hal_can.h"

// Global variables for CAN communication
static CAN_TxHeaderTypeDef dm_TxHeader;
static uint32_t dm_mailbox[3];

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;
extern motor_data_t pitch_motor;
extern motor_data_t yaw_motor;
extern EventGroupHandle_t gimbal_event_group;

void dm4310_motor_init(void)
{
// this function has been implemented in motor_config. should no longer be used

	#if PITCH_MOTOR_TYPE == TYPE_DM4310
		memset(&dm_pitch_motor, 0, sizeof(dm_pitch_motor));
		dm_pitch_motor.id = 0x81;
		dm_pitch_motor.ctrl.mode = 0;
		dm4310_enable(&hcan1, &dm_pitch_motor);
		vTaskDelay(3);
	#endif

	#if YAW_MOTOR_TYPE == TYPE_DM4310
		memset(&dm_yaw_motor, 0, sizeof(dm_yaw_motor));
		dm_yaw_motor.id = 0x61;
		dm_yaw_motor.ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
		dm4310_enable(&hcan2, &dm_yaw_motor);
		vTaskDelay(3);
	#endif
}

/** ************************************************************************
 * @brief:       dm4310_enable: Enables the control mode of the DM4310 motor
 * @param[in]:   hcan:    Pointer to a CAN_HandleTypeDef structure
 * @param[in]:   motor:   Pointer to a dm_motor_t structure, which contains configuration and control parameters for the motor
 * @retval:      void
 * @details:     Based on the motor's control mode, selects the appropriate mode and sends control commands via the CAN bus
 *               Supported control modes include position control, position-speed composite control, and speed control
 ************************************************************************ **/
void dm4310_enable(CAN_HandleTypeDef* hcan, dm_motor_t* motor)
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
* @brief:      	dm4310_disable: Disables the control mode of the DM4310 motor
* @param[in]:   hcan:    Pointer to a CAN_HandleTypeDef structure
* @param[in]:   motor:   Pointer to a dm_motor_t structure, which contains configuration and control parameters for the motor
* @retval:     	void
* @details:    	Based on the motor's control mode, selects the corresponding mode and sends a stop command via the CAN bus
*               Supported control modes include position control, position-speed composite control, and speed control
************************************************************************
**/
void dm4310_disable(CAN_HandleTypeDef* hcan, dm_motor_t *motor)
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
* @brief:      	dm4310_ctrl_send: Sends control commands to the DM4310
* @param[in]:   hcan:    Pointer to a CAN_HandleTypeDef structure
* @param[in]:   motor:   Pointer to a dm_motor_t structure, containing motor configuration and control parameters
* @retval:     	void
* @details:    	Sends corresponding commands to the DM4310 based on the control mode
*               Supported control modes include position control, position-speed composite control, and speed control
************************************************************************
**/
void dm4310_ctrl_send(CAN_HandleTypeDef* hcan, dm_motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case 0:
            mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, 
                    motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
            break;
        case 1:
            pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
            break;
        case 2:
            speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
            break;
        case 3:
            pos_force_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.tor_set);
            break;
    }    
}

/**
************************************************************************
* @brief:      	dm4310_set: Sets the target control parameters for DM4310
* @param[in]:   motor:   Pointer to a dm_motor_t structure, containing motor configuration and control parameters
* @retval:     	void
* @details:    	Sets the target control parameters for the DM4310, such as position, speed,
*               proportional gain (KP), derivative gain (KD), and torque
************************************************************************
**/
void dm4310_set(dm_motor_t *motor)
{
    motor->ctrl.kd_set  = motor->cmd.kd_set;
    motor->ctrl.kp_set  = motor->cmd.kp_set;
    motor->ctrl.pos_set = motor->cmd.pos_set;
    motor->ctrl.vel_set = motor->cmd.vel_set;
    motor->ctrl.tor_set = motor->cmd.tor_set;
}

/**
************************************************************************
* @brief:      	dm4310_clear: Clears the target control parameters for DM4310
* @param[in]:   motor:   Pointer to a dm_motor_t structure, containing motor configuration and control parameters
* @retval:     	void
* @details:    	Clears the target control parameters of the DM4310, including position, speed,
*               proportional gain (KP), derivative gain (KD), and torque
************************************************************************
**/
void dm4310_clear_para(dm_motor_t *motor)
{
//    motor->cmd.kd_set   = 0;
//    motor->cmd.kp_set   = 0;
//    motor->cmd.pos_set  = 0;
//    motor->cmd.vel_set  = 0;
//    motor->cmd.tor_set  = 0;
    
    motor->ctrl.kd_set  = 0;
    motor->ctrl.kp_set  = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
}

/**
************************************************************************
* @brief:      	dm4310_clear_err: Clears error state of DM4310
* @param[in]:   hcan: 	 Pointer to CAN controller structure
* @param[in]:   motor:   Pointer to motor structure
* @retval:     	void
* @details:    	Sends the appropriate error-clearing command based on the target control mode
************************************************************************
**/
void dm4310_clear_err(CAN_HandleTypeDef* hcan, dm_motor_t *motor)
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
* @brief:      	dm4310_fbdata: Retrieves feedback data from DM4310
* @param[in]:   motor:    Pointer to dm_motor_t structure, containing motor info and feedback buffer
* @param[in]:   rx_data:  Pointer to the received data buffer
* @retval:     	void
* @details:    	Extracts feedback information from received data, including motor ID,
*               status, position, speed, torque, and temperature
************************************************************************
**/
void dm4310_fbdata(dm_motor_t *motor, uint8_t *rx_data)
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
    motor->disconnect_time = get_microseconds();

	//initialise task switching variables
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken = pdFALSE;

    // Map feedback data based on motor ID
    if (motor->id == dm_yaw_motor.id) {
		xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b10,
				&xHigherPriorityTaskWoken);
        dmmapyawfbdata(motor);
    } else if (motor->id == dm_pitch_motor.id) {
		xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
				&xHigherPriorityTaskWoken);
        dmmappitchfbdata(motor);
    }
}

/**
************************************************************************
* @brief:       float_to_uint: Convert float to unsigned integer encoding
* @param[in]:   x_float:   The floating-point number to convert
* @param[in]:   x_min:     Minimum value of the mapping range
* @param[in]:   x_max:     Maximum value of the mapping range
* @param[in]:   bits:      Number of bits in the target unsigned integer
* @retval:      Encoded unsigned integer
* @details:     Maps the float x within the specified range [x_min, x_max]
*               and encodes it into an unsigned integer of the specified bit width
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
* @brief:       uint_to_float: Convert unsigned integer to float
* @param[in]:   x_int:     The unsigned integer to convert
* @param[in]:   x_min:     Minimum value of the mapping range
* @param[in]:   x_max:     Maximum value of the mapping range
* @param[in]:   bits:      Bit width of the unsigned integer
* @retval:      Resulting floating-point number
* @details:     Maps the unsigned integer x_int from a given range [x_min, x_max]
*               and converts it back into a float
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
* @brief:      	enable_motor_mode: Enables the motor control mode
* @param[in]:   hcan:     Pointer to CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID to specify target motor
* @param[in]:   mode_id:  Mode ID to specify the desired mode
* @retval:     	void
* @details:    	Sends command via CAN bus to start the specified mode on the target motor
************************************************************************
**/
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
    dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
    dm_TxHeader.StdId = motor_id + mode_id;
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    
//    for (int i = 0; i < 5; i++) {

		if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
		{
			// Abort any pending messages to free the mailbox
			HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
			HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
			HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
		}

		HAL_StatusTypeDef status = HAL_ERROR;
	if (status == HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox)){
			status = HAL_ERROR; // HIHIHIH
    }
}

/**
************************************************************************
* @brief:      	disable_motor_mode: Disables the motor control mode
* @param[in]:   hcan:     Pointer to CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID to specify target motor
* @param[in]:   mode_id:  Mode ID to specify the desired mode
* @retval:     	void
* @details:    	Sends command via CAN bus to start the specified mode on the target motor
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
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
* @brief:      	save_pos_zero: Save zero position
* @param[in]:   hcan:     Pointer to CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID to specify target motor
* @param[in]:   mode_id:  Mode ID to specify the desired mode
* @retval:     	void
* @details:    	Sends command via CAN bus to start the specified mode on the target motor
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
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
* @brief:      	clear_err: Clear error
* @param[in]:   hcan:     Pointer to CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID to specify target motor
* @param[in]:   mode_id:  Mode ID to specify the desired mode
* @retval:     	void
* @details:    	Sends command via CAN bus to start the specified mode on the target motor
************************************************************************
**/
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
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
* @brief:      	mit_ctrl: Motor control function in MIT mode
* @param[in]:   hcan:			Pointer to CAN_HandleTypeDef structure for CAN bus
* @param[in]:   motor_id:		Motor ID specifying the target motor
* @param[in]:   pos:			Target position
* @param[in]:   vel:			Target velocity
* @param[in]:   kp:				Proportional gain
* @param[in]:   kd:				Derivative gain
* @param[in]:   torq:			Target torque
* @retval:     	void
* @details:    	Sends a control frame to the motor in MIT control mode via CAN
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
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
 * @brief  Position and speed control for DM4310
 * @param  hcan: Pointer to CAN handle structure
 * @param  motor_id: Motor ID
 * @param  pos: Position setpoint
 * @param  vel: Velocity setpoint
 * @retval None
 */
void pos_speed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel)
{
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
    dm_TxHeader.RTR = CAN_RTR_DATA; //RTR

    uint8_t *pbuf, *vbuf;
    uint8_t data[8];
    
    dm_TxHeader.StdId = motor_id + POS_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;
    
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
* @brief:      	speed_ctrl: Velocity control function
* @param[in]:   hcan:		Pointer to CAN_HandleTypeDef structure for CAN bus
* @param[in]:   motor_id:	Motor ID specifying the target motor
* @param[in]:   vel:		Target velocity
* @retval:     	void
* @details:    	Sends velocity control command via CAN
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
    dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
    
    uint8_t *vbuf;
    uint8_t data[4];
    
    dm_TxHeader.StdId = motor_id + SPEED_MODE;
    vbuf = (uint8_t*)&vel;
    
    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);
    
    HAL_CAN_AddTxMessage(hcan, &dm_TxHeader, data, dm_mailbox);
}

void pos_force_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, uint16_t vel, uint16_t i)
{
    dm_TxHeader.DLC = 8; //The Data Bytes of the data. For the C620, it is 8 bytes of data.
    dm_TxHeader.IDE = CAN_ID_STD; //Standard CAN BUS transmission
    dm_TxHeader.RTR = CAN_RTR_DATA; //RTR
    
    uint8_t *pbuf, *vbuf, *ibuf;
    uint8_t data[8];
    
    dm_TxHeader.StdId = motor_id + POSI_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;
    ibuf = (uint8_t*)&i;
    
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

float dm_yaw_encoder_mod(float raw_angle) {
//    float mapped_angle = fmod(P_MAX + raw_angle, 2*P_MAX/P_ROUNDS);
//    return mapped_angle - P_MAX/P_ROUNDS;

    while (raw_angle > PI) { raw_angle -= 2 * PI; }
    while (raw_angle < -PI) { raw_angle += 2 * PI; }
    return raw_angle;
}

void dmmapyawfbdata(dm_motor_t *motor) {
	float adj_ang = dm_yaw_encoder_mod(motor->para.pos) - dm_yaw_motor.angle_data.center_ang;
	// maps from 0 to 2PI TO 0 to 8192
	//float mapped_value = (temp / (2 * PI)) * 8192;
	yaw_motor.angle_data.adj_ang = adj_ang;
	yaw_motor.raw_data.torque = motor->para.tor;
    dm_yaw_motor.angle_data.adj_ang = adj_ang;
}

void dmmappitchfbdata(dm_motor_t *motor) {
	float pos = motor->para.pos;
    pitch_motor.angle_data.adj_ang = pos - dm_pitch_motor.angle_data.center_ang;
    dm_pitch_motor.angle_data.adj_ang = pos - dm_pitch_motor.angle_data.center_ang;
}
