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
#include "INS_task.h"
#include "lqr_k.h"
#include "balancing_chassis_task.h"

Target target = {0};
extern LegPos leftLegPos, rightLegPos;
extern Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
StateVar stateVar;
//extern orientation_data_t balancing_imu;
extern motor_t motor[num];
extern remote_cmd_t g_remote_cmd;
extern motor_data_t g_can_motors[24];
//extern motor_t MF_motor[2];;


extern float dm_set_tor[4];
extern float mf_set_tor[2];
PID yawPID, rollPID;
PID legAnglePID, LlegLengthPID,RlegLengthPID;
PID spinPID;
int chassis_state = 0;
int ground_state = 0;
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
extern INS_t INS;
float spin_speed = 0.0f;
extern float filtered_v;
extern float filtered_x;
float LFN;
float RFN;
int robot_ready = 0;//1 ready 0 not ready
PID manual_left_F,manual_left_Tp,manual_right_F,manual_right_Tp;
float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
			{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;
double kRes[12] = {0}, k[2][6] = {0};
float LlqrOutT;
float LlqrOutTp;
float RlqrOutT;
float RlqrOutTp;
float F_gravity = 7.0f * 9.81f;

void Ctrl_Init()
{
	//robot main pid init
	PID_Init(&LlegLengthPID, 500, 0.0, 10.0, -100.0, 100.0);
	PID_Init(&RlegLengthPID, 500, 0.0, 10.0, -100.0, 100.0);
	PID_Init(&legAnglePID, 25, 0.5, 0.5, -5.0, 5.0);
	PID_Init(&rollPID, 250, 0.0, 1.0, -100.0, 100.0);
	PID_Init(&yawPID, 15.0, 1.0, 3.0, -2.5, 2.5);
	PID_Init(&spinPID, 1.0, 0.0, 0.1, -2.0, 2.0);
}

void Ctrl_TargetUpdateTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float speedSlopeStep = 1.0f; // Existing parameter for speed slope
    float speedCmdSlope = 0.01f; // New parameter for speed command slope limit

    while (1)
    {
        // Calculate desired speed command based on yaw angle
        float desiredSpeedCmd;
        if (target.yawAngle > 3.14 || target.yawAngle < -3.14)
        {
            desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * -2.5f;
        }
        else
        {
            desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * 2.5f;
        }

        // Limit the rate of change for the speed command
        if (desiredSpeedCmd == 0.0f)
        {
            // If desired command is zero, reset immediately
            target.speedCmd = 0.0f;
        }
        else if ((desiredSpeedCmd > 0 && target.speedCmd < 0) ||
                 (desiredSpeedCmd < 0 && target.speedCmd > 0))
        {
            // If the signs differ, directly set target.speedCmd to desiredSpeedCmd
            target.speedCmd = 0.0f;
        }
        else if (fabs(desiredSpeedCmd - target.speedCmd) < speedCmdSlope)
        {
            target.speedCmd = desiredSpeedCmd;
        }
        else
        {
            if (desiredSpeedCmd > target.speedCmd)
                target.speedCmd += speedCmdSlope;
            else
                target.speedCmd -= speedCmdSlope;
        }

        // Existing speed calculation logic
        spin_speed = ((float)g_remote_cmd.side_dial / 660) * 100.0f;

        // Calculate speed slope step based on leg length
        float legLength = (leftLegPos.length + rightLegPos.length) / 2;
        speedSlopeStep = -(legLength - 0.15f) * 0.03f + 1.0f;

        // Apply slope limitation to speed
        if (fabs(target.speedCmd - target.speed) < speedSlopeStep)
        {
            target.speed = target.speedCmd;
        }
        else
        {
            if (target.speedCmd - target.speed > 0)
                target.speed += speedSlopeStep;
            else
                target.speed -= speedSlopeStep;
        }

        // Calculate position target and limit to ±0.1m of current position
        target.position += target.speed * 0.005f;
        if (target.position - stateVar.x > 0.1f)
            target.position = stateVar.x + 0.1f;
        else if (target.position - stateVar.x < -0.1f)
            target.position = stateVar.x - 0.1f;

        // Limit speed target to ±1.5m/s of current speed
        if (target.speed - stateVar.dx > 1.5f)
            target.speed = stateVar.dx + 1.5f;
        else if (target.speed - stateVar.dx < -1.5f)
            target.speed = stateVar.dx - 1.5f;

        // Calculate yaw angle target
        vTaskDelayUntil(&xLastWakeTime, 5); // Update every 5ms
    }
}
int ground_detect(float LF, float LTP,float Ltheta,float LL0, float RF, float RTP,float Rtheta,float RL0) {
	LFN = LF*arm_cos_f32(Ltheta)+LTP*arm_sin_f32(Ltheta)/LL0;
	RFN = RF*arm_cos_f32(Rtheta)+RTP*arm_sin_f32(Rtheta)/RL0;
	if (LFN < 30.0f && RFN <30.0f){
		return 1;
	}else{
		return 0;
	}
}
int robot_check(){
	//

}
void state_update(){
	stateVar.phi = INS.Pitch;
	stateVar.dPhi = -INS.Gyro[1];
	stateVar.x = filtered_x;
	stateVar.dx = filtered_v;
	stateVar.Ltheta = leftLegPos.angle - M_PI_2 - INS.Pitch;
	stateVar.LdTheta = leftLegPos.dAngle - (-INS.Gyro[1]);
	stateVar.Rtheta = rightLegPos.angle - M_PI_2 - INS.Pitch;
	stateVar.RdTheta = rightLegPos.dAngle - (-INS.Gyro[1]);
	stateVar.legLength = (leftLegPos.length + rightLegPos.length) / 2;
	stateVar.dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;
}
void kill_chassis(){
	dm_set_tor[3] = 0;
	dm_set_tor[0] = 0;
	dm_set_tor[1] = 0;
	dm_set_tor[2] = 0;
	mf_set_tor[0] = 0;
	mf_set_tor[1] = 0;
	target.speed = 0.0f;
	target.position = stateVar.x;
}
void manual_set_PidInit(){
	PID_Init(&manual_left_F, 100, 0, 0, -30, 30); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_left_Tp, 0.05, 0.001, 0, -2, 2); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_right_F, 100, 0, 0, -30, 30); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_right_Tp, 0.05, 0.001, 0, -2, 2);
}
void manual_set_legPos(float angle, float legLength){
	float dt = 0.005f;
	float left_F_control;
	float left_Tp_control;
	float right_F_control;
	float right_Tp_control;
	float leftTorque[2];
	float rightTorque[2];
	PID_Compute(&manual_left_F, legLength, leftLegPos.length, dt, 0);
	PID_Compute(&manual_left_Tp, M_PI/2.0f + angle, leftLegPos.angle, dt , 0);
	PID_Compute(&manual_right_F, legLength, rightLegPos.length, dt, 0);
	PID_Compute(&manual_right_Tp, M_PI/2.0f + angle, rightLegPos.angle, dt, 0);
	left_F_control = manual_left_F.output;
	left_Tp_control = manual_left_Tp.output;
	right_F_control = -manual_right_F.output;
	right_Tp_control = -manual_right_Tp.output;
	leg_conv(left_F_control,left_Tp_control,leftJoint[0].angle,leftJoint[1].angle ,leftTorque);
	leg_conv(right_F_control,right_Tp_control,rightJoint[0].angle,rightJoint[1].angle,rightTorque);
	if (robot_ready == 1){
		dm_set_tor[3] = leftTorque[0];
		dm_set_tor[0] = leftTorque[1];
		dm_set_tor[1] = rightTorque[0];
		dm_set_tor[2] = rightTorque[1];
	}else{
		kill_chassis();
	}
}
void calculate_T_TP(int touching_ground){
	if (touching_ground == 1){
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 2; j++)
				k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
		}
	}else{
		memset(k, 0, sizeof(k));
		//    	        		k[1][0] = kRes[1] * -2;
		//    	        		k[1][1] = kRes[3] * -10;
	}

	float Lx[6] = {stateVar.Ltheta, stateVar.LdTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
	float Rx[6] = {stateVar.Rtheta, stateVar.RdTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
	Lx[2] -= target.position;
	Lx[3] -= target.speed;
	Rx[2] -= target.position;
	Rx[3] -= target.speed;
	LlqrOutT = k[0][0] * Lx[0] + k[0][1] * Lx[1] + k[0][2] * Lx[2] + k[0][3] * Lx[3] + k[0][4] * Lx[4] + k[0][5] * Lx[5];
	LlqrOutTp = k[1][0] * Lx[0] + k[1][1] * Lx[1] + k[1][2] * Lx[2] + k[1][3] * Lx[3] + k[1][4] * Lx[4] + k[1][5] * Lx[5];
	RlqrOutT = k[0][0] * Rx[0] + k[0][1] * Rx[1] + k[0][2] * Rx[2] + k[0][3] * Rx[3] + k[0][4] * Rx[4] + k[0][5] * Rx[5];
	RlqrOutTp = k[1][0] * Rx[0] + k[1][1] * Rx[1] + k[1][2] * Rx[2] + k[1][3] * Rx[3] + k[1][4] * Rx[4] + k[1][5] * Rx[5];
}
void gimbal_auto_front(){ //find shortest distance to align robot gimbal and body
	if(g_can_motors[19].angle_data.adj_ang > M_PI_2){
		target.yawAngle = M_PI;
	}else if(g_can_motors[19].angle_data.adj_ang < -M_PI_2){
		target.yawAngle = -M_PI;
	}else{
		target.yawAngle=0;
	}
}
void balancing_chassis_task(void *argument) {
	const float wheelRadius = 0.0925f; //m，车轮半径
	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float leftForce;
	float rightForce;
	float leftTp;
	float rightTp;
	float leftJointTorque[2]={0};
	float rightJointTorque[2]={0};
	const float legMass = 0.8f; //kg，腿部质量
	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.17f;
	target.speed = 0.0f;
	target.position = stateVar.x;
	target.floating_legLength = 0.22f;
	target.min_legLength = 0.12f;
	float dt = 0.005f;
	Ctrl_Init();
	manual_set_PidInit();
	osDelay(2000);
    while (1) {
    	robot_ready = robot_check();
    	state_update();
    	lqr_k(stateVar.legLength, kRes);
    	if (g_remote_cmd.right_switch != 3){
    		chassis_state = 0;
    	}
    	gimbal_auto_front(); //align gimbal 0 or 180
    	switch (chassis_state) {
    	        case 0: // robot die
    	        	//do robot checking
    	        	kill_chassis();
    	           	target.position = stateVar.x;//reset target pos
    	           	ground_state = 0; //on ground
    	           	if (robot_ready == 1){
    	           		chassis_state = 1;
    	           		break;
    	           	}else{// try to reconnect

    	           		chassis_state = 0;
    	           		break;
    	           	}
    	            break;

    	        case 1: // leg positioning
    	        	manual_set_legPos(0,0.12);
    	        	if ((leftLegPos.angle >1.3 && leftLegPos.angle < 1.7) &&
    	        			(leftLegPos.length >0.1 && leftLegPos.length < 0.15)&&
							(rightLegPos.angle >1.3 && rightLegPos.angle < 1.7)&&
							(rightLegPos.length >0.1 && rightLegPos.length < 0.15)){
    	        		chassis_state = 2;
    	        		break;
    	        	}
    	            break;

    	        case 2: // standing with min leglength
    	        	target.speed = 0.0f;
    	        	target.position = stateVar.x;
    	        	calculate_T_TP(1);// 1 touching ground 0 not touching ground

    	        	//PID_Compute(&yawPID, target.yawAngle, g_can_motors[19].angle_data.adj_ang,0.005,0);
    	        	//PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm,0.005,0);
    	        	mf_set_tor[0] = -LlqrOutT * lqrTRatio;
    	        	mf_set_tor[1] = -RlqrOutT * lqrTRatio;
//    	        	if (fabs(spin_speed)>0){
//    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;
//    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;
//    	        	}else{
//    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
//    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
//    	        	}

    	        	PID_Compute(&LlegLengthPID, target.min_legLength, leftLegPos.length,dt,0);
    	        	PID_Compute(&RlegLengthPID, target.min_legLength, rightLegPos.length,dt,0);
    	        	PID_Compute(&rollPID, target.rollAngle, INS.Roll,dt,0);
    	        	PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
//    	        	double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
//    	        	double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;

    	        	leftForce = LlegLengthPID.output + F_gravity +rollPID.output;
    	        	rightForce = RlegLengthPID.output + F_gravity -rollPID.output;
    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
    	        		leftForce -= (leftLegPos.length - 0.25f) * 100.0f;
    	        	if(rightLegPos.length > 0.25f)
    	        		rightForce -= (rightLegPos.length - 0.25f) * 100.0f;
    	        	leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
    	        	rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));

    	        	leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	        	leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

//    	        	leftF_check = leftForce;
//    	        	leftTp_check = leftTp;
//    	        	rightF_check = rightForce;
//    	        	rightTp_check = rightTp;
//    	        	l1 = leftJointTorque[0];
//    	        	l4 = leftJointTorque[1];
//    	        	r1 = rightJointTorque[0];
//    	        	r4 = rightJointTorque[1];
    	        	dm_set_tor[3] = leftJointTorque[0];
    	        	dm_set_tor[0] = leftJointTorque[1];
    	        	dm_set_tor[1] = -rightJointTorque[0];
    	        	dm_set_tor[2] = -rightJointTorque[1];

    	        	if(){// if robot in steady state > 0.5 sec
    	        		chassis_state = 3;
    	        		break;
    	        	}
    	            break;
    	        case 3: // robot standing with target leglength

    	        	calculate_T_TP(1);// 1 touching ground 0 not touching ground
    	        	if(ground_state != 0) //if robot not touching ground
    	        	{
    	        		chassis_state = 4;
    	        		break;
    	        	}

    	        	PID_Compute(&yawPID, target.yawAngle, g_can_motors[19].angle_data.adj_ang,0.005,0);
    	        	PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm,0.005,0);

    	        	if (fabs(spin_speed)>0){
    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;
    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;
    	        	}else{
    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
    	        	}


    	        	PID_Compute(&LlegLengthPID, target.legLength, leftLegPos.length,dt,0);
    	        	PID_Compute(&RlegLengthPID, target.legLength, rightLegPos.length,dt,0);
    	        	PID_Compute(&rollPID, target.rollAngle, INS.Roll,dt,0);
    	        	PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
    	        	//    	        	double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
    	        	//    	        	double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;

    	        	leftForce = LlegLengthPID.output + F_gravity +rollPID.output;
    	        	rightForce = RlegLengthPID.output + F_gravity -rollPID.output;
    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
    	        		leftForce -= (leftLegPos.length - 0.25f) * 100.0f;
    	        	if(rightLegPos.length > 0.25f)
    	        		rightForce -= (rightLegPos.length - 0.25f) * 100.0f;
    	        	leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
    	        	rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));

    	        	leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	        	leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

    	        	//    	        	leftF_check = leftForce;
    	        	//    	        	leftTp_check = leftTp;
    	        	//    	        	rightF_check = rightForce;
    	        	//    	        	rightTp_check = rightTp;
    	        	//    	        	l1 = leftJointTorque[0];
    	        	//    	        	l4 = leftJointTorque[1];
    	        	//    	        	r1 = rightJointTorque[0];
    	        	//    	        	r4 = rightJointTorque[1];
    	        	dm_set_tor[3] = leftJointTorque[0];
    	        	dm_set_tor[0] = leftJointTorque[1];
    	        	dm_set_tor[1] = -rightJointTorque[0];
    	        	dm_set_tor[2] = -rightJointTorque[1];

    	        	ground_state = ground_detect(leftForce,leftTp,stateVar.Ltheta,leftLegPos.length
    	        			,rightForce,rightTp,stateVar.Rtheta,rightLegPos.length);

    	            break;
    	        case 4: //robot floating
    	        	calculate_T_TP(0);// not touching ground
    	        	//    	        		k[1][0] = kRes[1] * -2;
    	        	//    	        		k[1][1] = kRes[3] * -10;
    	        	mf_set_tor[0] = 0;
    	        	mf_set_tor[1] = 0;
    	        	dm_set_tor[3] = leftJointTorque[0];
    	        	dm_set_tor[0] = leftJointTorque[1];
    	        	dm_set_tor[1] = -rightJointTorque[0];
    	        	dm_set_tor[2] = -rightJointTorque[1];

    	        	break;
    	        case 5: //cushioning


    	        	break;
    	        default: // Default case for invalid input
    	}
        vTaskDelay(5);
    }
}

