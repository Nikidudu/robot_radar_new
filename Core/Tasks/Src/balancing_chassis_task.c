/*
 * balancing_chassis_task.c
 *
 *  Created on: Dec 10, 2024
 *      Author: YI MING
 */
#include <math.h>
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "balancing_imu_task.h"
#include "dm4310_drv.h"
#include "PID.h"
#include "leg_task.h"

Target target = {0, 0, 0, 0, 0, 0, 0.15f};
extern LegPos leftLegPos, rightLegPos;
extern Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
StateVar stateVar;
extern orientation_data_t balancing_imu;
extern motor_t motor[num];
extern remote_cmd_t g_remote_cmd;
float left_F_control;
float left_Tp_control;
float right_F_control;
float right_Tp_control;
float leftTorque[2];
float rightTorque[2];
extern float dm_set_tor[4];
int chassis_state = 0;

void balancing_chassis_task(void *argument) {
	const float wheelRadius = 0.09f; //m，车轮半径
	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
			{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;
	const float legMass = 0.01f; //kg，腿部质量
	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.15f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	float dt = 0.005f;


	PID left_F;
	PID left_Tp;
	PID right_F;
	PID right_Tp;
	osDelay(2000);
    while (1) {
    	stateVar.phi = balancing_imu.pit;
    	stateVar.dPhi = balancing_imu.pit_speed;
    	stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
    	stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
    	stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - balancing_imu.pit;
    	stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - balancing_imu.pit_speed;
    	float legLength = (leftLegPos.length + rightLegPos.length) / 2;
    	float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

    	switch (chassis_state) {
    	        case 0: // leg move to position
    	        	if (g_remote_cmd.right_switch == 3){
    	        		PID_Init(&left_F, 100, 0, 0, -30, 30); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
    	        		PID_Init(&left_Tp, 0.05, 0.001, 0, -2, 2); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
    	        		PID_Init(&right_F, 100, 0, 0, -30, 30); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
    	        		PID_Init(&right_Tp, 0.05, 0.001, 0, -2, 2);
    	        		PID_Compute(&left_F, 0.15, leftLegPos.length, dt, 0);
    	        		PID_Compute(&left_Tp, M_PI/2.0f, leftLegPos.angle, dt , 0);
    	        		PID_Compute(&right_F, 0.15, rightLegPos.length, dt, 0);
    	        		PID_Compute(&right_Tp, M_PI/2.0f, rightLegPos.angle, dt, 0);
    	        		left_F_control = left_F.output;
    	        		//    	left_F_control = 0;
    	        		left_Tp_control = left_Tp.output;
    	        		right_F_control = -right_F.output;
    	        		//		right_F_control = 0;
    	        		right_Tp_control = -right_Tp.output;
    	        		leg_conv(left_F_control,left_Tp_control,leftJoint[0].angle,leftJoint[1].angle ,leftTorque);
    	        		leg_conv(right_F_control,right_Tp_control,rightJoint[0].angle,rightJoint[1].angle,rightTorque);

    	        		dm_set_tor[3] = leftTorque[0];
    	        		dm_set_tor[0] = leftTorque[1];
    	        		dm_set_tor[1] = rightTorque[0];
    	        		dm_set_tor[2] = rightTorque[1];
    	        	}else{
    	        		dm_set_tor[3] = 0;
    	        		dm_set_tor[0] = 0;
    	        		dm_set_tor[1] = 0;
    	        		dm_set_tor[2] = 0;
    	        	}
    	            break;

    	        case 1: // Case for 'Stop'
    	            break;

    	        case 2: // Case for 'Pause'
    	            break;

    	        case 3: // Case for 'Exit'
    	            break;

    	        default: // Default case for invalid input
    	}
        vTaskDelay(5);
    }
}
