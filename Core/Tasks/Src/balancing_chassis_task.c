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
extern motor_data_t g_can_motors[24];
//extern motor_t MF_motor[2];;
float left_F_control;
float left_Tp_control;
float right_F_control;
float right_Tp_control;
float leftTorque[2];
float rightTorque[2];
extern float dm_set_tor[4];
extern float mf_set_tor[2];
PID yawPID, rollPID;
PID legAnglePID, legLengthPID;
int chassis_state = 0;
int robot_ground = 0; //0 unknown 1 touching ground 2 flying
float check_T;
float check_Tp;
double leftF_check;
double leftTp_check;
double rightF_check;
double rightTp_check;
double l1;
double l4;
double r1;
double r4;
float BODY_MASS = 20.0f;
float n = 0.5f;
float LEG_MASS = 0.8f;
float yaw_angle_offset = 2.14f;
float max_Tp = 5.0f;

void Ctrl_Init()
{
	//初始化各个PID参数
//	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
	PID_Init(&legLengthPID, 1200, 0.0, 5.0, -30.0, 30.0);
//	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
	PID_Init(&legAnglePID, 25, 0.5, 0.5, -5.0, 5.0);
//	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);
	PID_Init(&rollPID, 300, 0.0, 10.0, -60.0, 60.0);
	PID_Init(&yawPID, 8.0, 0.1, 1.0, -1.1, 1.1);
}

void Ctrl_TargetUpdateTask()
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 1.0f;
	while(1){
			if (target.yawAngle > 3.14 || target.yawAngle < -3.14){
				target.speedCmd = ((float)g_remote_cmd.left_y/660)*-1.5f;
			}else{
				target.speedCmd = ((float)g_remote_cmd.left_y/660)*1.5f;
			}

			//target.yawAngle = g_can_motors[19].angle_data.adj_ang;
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
			float legLength = (leftLegPos.length + rightLegPos.length) / 2;
			speedSlopeStep = -(legLength - 0.15f) * 0.03f + 1.0f;

			//计算速度斜坡，斜坡值更新到target.speed
			if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
				target.speed = target.speedCmd;
			else
			{
				if(target.speedCmd - target.speed > 0)
					target.speed += speedSlopeStep;
				else
					target.speed -= speedSlopeStep;
			}

			//计算位置目标，并限制在当前位置的±0.2m内
			target.position += target.speed * 0.005f;
			if(target.position - stateVar.x > 0.2f)
				target.position = stateVar.x + 0.2f;
			else if(target.position - stateVar.x < -0.2f)
				target.position = stateVar.x - 0.2f;

			//限制速度目标在当前速度的±0.3m/s内
			if(target.speed - stateVar.dx > 0.7f)
				target.speed = stateVar.dx + 0.7f;
			else if(target.speed - stateVar.dx < -0.7f)
				target.speed = stateVar.dx - 0.7f;

			//计算yaw方位角目标
			vTaskDelayUntil(&xLastWakeTime, 5); //每4ms更新一次
	}
}

void balancing_chassis_task(void *argument) {
	const float wheelRadius = 0.0925f; //m，车轮半径
	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
			{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;
	const float legMass = 0.8f; //kg，腿部质量
	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.15f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	float dt = 0.005f;
	Ctrl_Init();
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
    	stateVar.Ltheta = leftLegPos.angle - M_PI_2 - balancing_imu.pit;
    	stateVar.LdTheta = leftLegPos.dAngle - balancing_imu.pit_speed;
    	stateVar.Rtheta = rightLegPos.angle - M_PI_2 - balancing_imu.pit;
    	stateVar.RdTheta = rightLegPos.dAngle - balancing_imu.pit_speed;
    	double legLength = (leftLegPos.length + rightLegPos.length) / 2;
    	double dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

    	double kRes[12] = {0}, k[2][6] = {0};
    	lqr_k(legLength, kRes);

    	switch (chassis_state) {
    	        case 0: // leg move to position
    	        	//do robot checking
    	        	dm_set_tor[3] = 0;
    	        	dm_set_tor[0] = 0;
    	        	dm_set_tor[1] = 0;
    	        	dm_set_tor[2] = 0;
    	           	mf_set_tor[0] = 0;
    	           	mf_set_tor[1] = 0;
    	        	chassis_state = 1;
    	        	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
    	            break;

    	        case 1: // leg positioning
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
    	        		if ((leftLegPos.angle >1.3 && leftLegPos.angle < 1.7) &&
    	        				(leftLegPos.length >0.1 && leftLegPos.length < 0.15)&&
								(rightLegPos.angle >1.3 && rightLegPos.angle < 1.7)&&
								(rightLegPos.length >0.1 && rightLegPos.length < 0.15)){
    	        			chassis_state = 2;
    	        			robot_ground = 1;
    	        		}
    	        	}else{
    	        		dm_set_tor[3] = 0;
    	        		dm_set_tor[0] = 0;
    	        		dm_set_tor[1] = 0;
    	        		dm_set_tor[2] = 0;
    	        		mf_set_tor[0] = 0;
        	           	mf_set_tor[1] = 0;
        	           	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
    	        	}
    	            break;

    	        case 2: // standing
    	        	for (int i = 0; i < 6; i++)
    	        	{
    	        		for (int j = 0; j < 2; j++)
    	        			k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
    	        	}
    	        	//准备状态变量
    	        	float Lx[6] = {stateVar.Ltheta, stateVar.LdTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
    	        	float Rx[6] = {stateVar.Rtheta, stateVar.RdTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
    	        	//与给定量作差
    	        	Lx[2] -= target.position;
    	        	Lx[3] -= target.speed;
    	        	Rx[2] -= target.position;
    	        	Rx[3] -= target.speed;


    	        	//check_x = x[2];
    	        	//矩阵相乘，计算LQR输出
    	        	float LlqrOutT = k[0][0] * Lx[0] + k[0][1] * Lx[1] + k[0][2] * Lx[2] + k[0][3] * Lx[3] + k[0][4] * Lx[4] + k[0][5] * Lx[5];
    	        	float LlqrOutTp = k[1][0] * Lx[0] + k[1][1] * Lx[1] + k[1][2] * Lx[2] + k[1][3] * Lx[3] + k[1][4] * Lx[4] + k[1][5] * Lx[5];
    	        	float RlqrOutT = k[0][0] * Rx[0] + k[0][1] * Rx[1] + k[0][2] * Rx[2] + k[0][3] * Rx[3] + k[0][4] * Rx[4] + k[0][5] * Rx[5];
    	        	float RlqrOutTp = k[1][0] * Rx[0] + k[1][1] * Rx[1] + k[1][2] * Rx[2] + k[1][3] * Rx[3] + k[1][4] * Rx[4] + k[1][5] * Rx[5];
    	        	check_T = LlqrOutT;
    	        	check_Tp = LlqrOutTp;



    	        	if(g_can_motors[19].angle_data.adj_ang > M_PI_2){
    	        		target.yawAngle = M_PI;
    	        	}else if(g_can_motors[19].angle_data.adj_ang < -M_PI_2){
    	        		target.yawAngle = -M_PI;
    	        	}else{
    	        		target.yawAngle=0;
    	        	}

    	        	PID_Compute(&yawPID, target.yawAngle, g_can_motors[19].angle_data.adj_ang,0.005,0);
    	        	//if robot not floating output motor else change state to 3

    	        	if (robot_ground == 1){
    	        		if (g_remote_cmd.right_switch == 3)
    	        		{
//    	        			g_can_motors[14].torque = -lqrOutT * lqrTRatio + yawPID.output;
//    	        			g_can_motors[12].torque = -lqrOutT * lqrTRatio - yawPID.output;
    	        			mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
    	        			mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
//    	        			MF_motor[0].ctrl.tor_set = 0;
//    	        			MF_motor[1].ctrl.tor_set = 0;
    	        		}else{
    	        			mf_set_tor[0] = 0;
    	    	           	mf_set_tor[1] = 0;
    	        			chassis_state = 1;
    	        		}
    	        	}
    	        	PID_Compute(&legLengthPID, target.legLength, legLength,0.005,0);
    	        	PID_Compute(&rollPID, target.rollAngle, balancing_imu.rol,0.005,0);
    	        	PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,0.005,0.01);
//    	        	double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
//    	        	double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;
    	        	float F_gravity = 7.0f * 9.81f;
    	        	float leftForce = legLengthPID.output + F_gravity +rollPID.output;
    	        	float rightForce = legLengthPID.output + F_gravity -rollPID.output;
    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
    	        		leftForce -= (leftLegPos.length - 0.25f) * 10.0f;
    	        	if(rightLegPos.length > 0.25f)
    	        		rightForce -= (rightLegPos.length - 0.25f) * 10.0f;
    	        	float leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
    	        	float rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
//    	        	float leftTp = legAnglePID.output + (leftLegPos.length);
//    	        	float rightTp = -(legAnglePID.output + (rightLegPos.length));
//    	        	leftForce = leftForce/9.0f; //motor gear ratio
//    	        	rightForce = rightForce/9.0f;
//    	        	leftTp = leftTp/9.0f;
//    	        	rightTp = rightTp/9.0f;
    	        	float leftJointTorque[2]={0};
    	        	leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	        	float rightJointTorque[2]={0};
    	        	leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);
    	        	leftF_check = leftForce;
    	        	leftTp_check = leftTp;
    	        	rightF_check = rightForce;
    	        	rightTp_check = rightTp;
    	        	l1 = leftJointTorque[0];
    	        	l4 = leftJointTorque[1];
    	        	r1 = rightJointTorque[0];
    	        	r4 = rightJointTorque[1];
    	        	dm_set_tor[3] = leftJointTorque[0];
    	        	dm_set_tor[0] = leftJointTorque[1];
    	        	dm_set_tor[1] = -rightJointTorque[0];
    	        	dm_set_tor[2] = -rightJointTorque[1];
    	            break;
    	        case 3: // floating
    	        	memset(k, 0, sizeof(k));
    	        	k[1][0] = kRes[1] * -2;
    	        	k[1][1] = kRes[3] * -10;

    	            break;

    	        default: // Default case for invalid input
    	}
        vTaskDelay(5);
    }
}

