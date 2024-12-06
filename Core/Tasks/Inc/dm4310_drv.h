#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200
#define POSI_MODE		  0x300

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	Motor4,
	Motor5,
	Motor6,
	num
} motor_num;


// ����ش���Ϣ�ṹ��
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	float temperature;
	float torque;
	float speed;
	float encoder_angle;
}motor_fbpara_t;

// ����������ýṹ��
typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	int16_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;
}motor_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm4310_ctrl_send(CAN_HandleTypeDef* hcan, motor_t *motor);
void dm4310_enable(CAN_HandleTypeDef* hcan, motor_t* motor);
void dm4310_disable(CAN_HandleTypeDef* hcan, motor_t *motor);
void dm4310_set(motor_t *motor);
void dm4310_clear_para(motor_t *motor);
void dm4310_clear_err(CAN_HandleTypeDef* hcan, motor_t *motor);
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data);
void dm4310_motor_init(void);
void enableMFMotor(CAN_HandleTypeDef* hcan,int id);
void disableMFMotor(CAN_HandleTypeDef* hcan,int id);
void MFspeed_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);
void MF_fbdata(motor_t *motor, uint8_t *rx_data);
void MFtorque_command(CAN_HandleTypeDef* hcan, uint16_t motor_id, float desired_torque);

void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float _vel);
void pos_force_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, uint16_t vel, uint16_t i);
	
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

#ifdef __cplusplus
}
#endif

#endif /* __DM4310_DRV_H__ */

