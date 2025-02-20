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
PID LcushionPID,RcushionPID;
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
float LFTP;
float RFTP;
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
float F_gravity = 8.0f * 9.81f;
float LFN_filtered = 0.0f;
float RFN_filtered = 0.0f;
float Average_FN = 0.0f;
float Average_Accel = 0.0f;
const float alpha = 0.5f; // Smoothing factor (adjust as needed)
float check_speed;
float check_x;
int spin_toggle = 0;
int last_jump_state = 0;
int jump_state = 0;
float jump_start_time;
float jump_now_time;
float k_jump_time = 0.185f;
float k_retract_time = 0.11f;
uint32_t jump_time_l = 0;
uint32_t jump_time_r = 0;
int jump_state_l = 0;
int jump_state_r = 0;
extern uint8_t joint_motor_online;
uint8_t gimbal_direction_toggle = 0;
float filtered_adj_ang = 0.0f;
PID angleFilterPID;  // PID instance for filtering the angle measurement
int error6900 = 0;
int error2777 = 0;
int staircase_toggle = 0;
uint8_t staircase_state = 0;
uint8_t staircase_detect = 0; //if 1 mean robot hit the staircase edge


void Ctrl_Init()
{
	//robot main pid init
	PID_Init(&LlegLengthPID, 500, 0.0, 150.0, -200.0, 200.0);
	PID_Init(&RlegLengthPID, 500, 0.0, 150.0, -200.0, 200.0);
	PID_Init(&LcushionPID, 800, 0.0, 150.0, -200.0, 200.0);
	PID_Init(&RcushionPID, 800, 0.0, 150.0, -200.0, 200.0);
	PID_Init(&legAnglePID, 20, 0.0, 1.0, -5.0, 5.0);
	PID_Init(&rollPID, 500, 0.0, 2.0, -200.0, 200.0);
	PID_Init(&yawPID, 0.007, 0.0005, 0.0015, -5.0, 5.0);
	PID_Init(&spinPID, 3.0, 0.0, 0.1, -2.0, 2.0);
	PID_Init(&angleFilterPID, 0.3f, 0.0f, 0.0f, -1.0f, 1.0f);
	filtered_adj_ang = g_can_motors[19].angle_data.adj_ang;

}

#define MAX_ANGLE 8000
#define HALF_RANGE (MAX_ANGLE / 2)
int computeError(int current, int target) {
    int error = ((current - target + HALF_RANGE) % MAX_ANGLE) - HALF_RANGE;
    return error;
}

void Ctrl_TargetUpdateTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float speedSlopeStep = 0.5f; // Existing parameter for speed slope
    float speedCmdSlope = 0.01f; // New parameter for speed command slope limit

    while (1)
    {
        // Calculate desired speed command based on yaw angle
        float desiredSpeedCmd;
        if (fabs(error6900) < fabs(error2777)){
        	desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * 2.5f;
        }else{
        	desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * -2.5f;
        }
//        if (target.yawAngle > 3.14 || target.yawAngle < -3.14)
//        {
//            desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * -3.5f;
//        }
//        else
//        {
//            desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * 3.5f;
//        }

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
        if ((float)g_remote_cmd.side_dial > 0){
        	spin_speed = ((float)g_remote_cmd.side_dial / 660) * 90.0f;
        }else{
        	spin_speed = 0;
        }
//        if (g_remote_cmd.side_dial == -660){
//        	jump_state = 1;
//        }
        if (chassis_state == 3 || chassis_state == 6){
        	if (g_remote_cmd.side_dial == -660 && staircase_toggle != -660) {
        		// Toggle staircase_state between 0 and 1.
        		staircase_state = !staircase_state;
        		staircase_toggle = -660;  // Prevent further toggling until the button is released.
        	} else if (g_remote_cmd.side_dial > -360) {
        		// Reset the toggle flag when the dial is not at -660.
        		staircase_toggle = 0;
        	}
        }


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
        if (target.position - stateVar.x > 0.15f)
            target.position = stateVar.x + 0.15f;
        else if (target.position - stateVar.x < -0.15f)
            target.position = stateVar.x - 0.15f;

        // Limit speed target to ±1.5m/s of current speed
        if (target.speed - stateVar.dx > 1.5f)
            target.speed = stateVar.dx + 1.5f;
        else if (target.speed - stateVar.dx < -1.5f)
            target.speed = stateVar.dx - 1.5f;
        target.legLength = 0.19f + ((float)g_remote_cmd.left_x / 660)*0.07f;
        // Calculate yaw angle target
        vTaskDelayUntil(&xLastWakeTime, 5); // Update every 5ms
    }
}
float threshold_variation;
int ground_detect(float LF, float LTP, float Ltheta, float LL0, float RF, float RTP, float Rtheta, float RL0) {
    // Calculate LFN and RFN
	threshold_variation = fabs(sin((stateVar.Ltheta + stateVar.Rtheta)/2)*50);
    LFN = LF * arm_cos_f32(Ltheta) + LTP * arm_sin_f32(Ltheta) / LL0;
    RFN = RF * arm_cos_f32(Rtheta) + RTP * arm_sin_f32(Rtheta) / RL0;

    // Apply low-pass filter
    LFN_filtered = alpha * LFN + (1.0f - alpha) * LFN_filtered;
    RFN_filtered = alpha * RFN + (1.0f - alpha) * RFN_filtered;

    // Use filtered values for ground detection
    if (ground_state == 0){
    	if (LFN_filtered < (0.0f-threshold_variation) && RFN_filtered < (0.0f-threshold_variation)) {
    		return 1;
    	} else {
    		return 0;
    	}
    }else{
    	if (LFN_filtered < (50.0f-threshold_variation) && RFN_filtered < (50.0f-threshold_variation)) {
    		return 1;
    	} else {
    		return 0;
    	}
    }

}

int ground_detect_staircase(float LF, float LTP, float Ltheta, float LL0, float RF, float RTP, float Rtheta, float RL0) {
    // Calculate LFN and RFN
	//threshold_variation = fabs(sin((stateVar.Ltheta + stateVar.Rtheta)/2)*50);
    LFN = LF * arm_cos_f32(Ltheta) + LTP * arm_sin_f32(Ltheta) / LL0;
    RFN = RF * arm_cos_f32(Rtheta) + RTP * arm_sin_f32(Rtheta) / RL0;

    // Apply low-pass filter
    LFN_filtered = alpha * LFN + (1.0f - alpha) * LFN_filtered;
    RFN_filtered = alpha * RFN + (1.0f - alpha) * RFN_filtered;

    Average_FN = (LFN_filtered + RFN_filtered)/2.0f;

    //if (Average_FN < (80.0f) && INS.Accel[0] < (-10.0f) && target.speedCmd > 0.1f) {
    if (INS.Accel[0] < (-10.0f) && target.speedCmd > 0.1f) {
    	return 1;
    } else {
    	return 0;
    }
}
int robot_check(){
	//
	return 1;
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
	PID_Init(&manual_left_F, 400, 0, 5, -70, 70); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_left_Tp, 20.0, 0, 1.0, -50, 50); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_right_F, 400, 0, 5, -70, 70); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
	PID_Init(&manual_right_Tp, 20.0, 0, 1.0, -50, 50);
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
		k[1][0] = kRes[1]*1.5f;
		k[1][1] = kRes[3]*1.5f;
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
	if(fabs(g_can_motors[19].angle_data.adj_ang) > M_PI_2 && gimbal_direction_toggle == 0){
		gimbal_direction_toggle = 1;
	}else if(fabs(g_can_motors[19].angle_data.adj_ang) > M_PI_2 && gimbal_direction_toggle == 1){
		gimbal_direction_toggle = 0;
	}
	if (gimbal_direction_toggle == 1){
		g_can_motors[19].angle_data.center_ang = 2777;
	}else{
		g_can_motors[19].angle_data.center_ang = 6905;
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
	target.legLength = 0.19f;
	target.speed = 0.0f;
	target.position = stateVar.x;
	target.floating_legLength = 0.35f;
	target.min_legLength = 0.1f;
	float dt = 0.005f;
	Ctrl_Init();
	manual_set_PidInit();
	osDelay(2000);
    while (1) {
    	robot_ready = robot_check();
    	state_update();
    	lqr_k(stateVar.legLength, kRes);
//    	if (g_remote_cmd.right_switch != 3 ||
//    			motor[Motor1].para.online != 1||
//				motor[Motor2].para.online != 1||
//				motor[Motor3].para.online != 1||
//				motor[Motor4].para.online !=1){
//    		chassis_state = 0;
//    	}
    	if (g_remote_cmd.right_switch != 3){// || joint_motor_online == 0){
    		chassis_state = 0;
    	}
    	gimbal_auto_front(); //align gimbal 0 or 180
    	F_gravity = fabs(cos((stateVar.Ltheta + stateVar.Rtheta)/2)*8.0f*9.81f);
    	switch (chassis_state) {
    	        case 0: // robot die
    	        	//do robot checking
    	        	kill_chassis();
    	        	jump_state = 0;
    	        	staircase_state = 0;
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
    	        	staircase_state = 0;
    	        	target.position = stateVar.x;
    	        	calculate_T_TP(1);// 1 touching ground 0 not touching ground
    	        	error6900 = computeError(g_can_motors[19].raw_data.angle[0], 6900);
    	        	error2777 = computeError(g_can_motors[19].raw_data.angle[0], 2777);

    	        	//PID_Compute(&yawPID, target.yawAngle, g_can_motors[19].angle_data.adj_ang,0.005,0);
    	        	//PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm,0.005,0);
//    	        	mf_set_tor[0] = -LlqrOutT * lqrTRatio;
//    	        	mf_set_tor[1] = -RlqrOutT * lqrTRatio;
    	        	if (fabs(error6900) < fabs(error2777)){
    	        		PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
    	        	}else{
    	        		PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
    	        	}
    	        	mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
    	        	mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
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
//    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
//    	        		leftForce -= (leftLegPos.length - 0.25f) * 100.0f;
//    	        	if(rightLegPos.length > 0.25f)
//    	        		rightForce -= (rightLegPos.length - 0.25f) * 100.0f;
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

    	        	static TickType_t steadyStateStartTime = 0; // To store when steady state starts
    	        	static int isSteadyStateTimerActive = 0;   // 0 means inactive, 1 means active

    	        	// Define the range checks for steady state
    	        	int isSteadyState = (fabs(stateVar.phi) < 0.1) &&
    	        	                    (fabs(stateVar.Ltheta) < 0.1) &&
    	        	                    (fabs(stateVar.Rtheta) < 0.1);

    	        	if (isSteadyState) {
    	        	    if (!isSteadyStateTimerActive) {
    	        	        // Start the timer when the steady state condition is first met
    	        	        steadyStateStartTime = xTaskGetTickCount();
    	        	        isSteadyStateTimerActive = 1;
    	        	    } else if ((xTaskGetTickCount() - steadyStateStartTime) * portTICK_PERIOD_MS >= 500) {
    	        	        // If the steady state condition persists for 500 ms
    	        	        chassis_state = 3;
    	        	        break;
    	        	    }
    	        	} else {
    	        	    // Reset the timer if steady state condition is broken
    	        	    isSteadyStateTimerActive = 0;
    	        	}
    	            break;
    	        case 3: // robot standing with target leglength
    	        	staircase_detect = 0;
    	        	calculate_T_TP(1);// 1 touching ground 0 not touching ground
    	        	if(ground_state != 0) //if robot not touching ground
    	        	{
    	        		chassis_state = 4;
    	        		break;
    	        	}
    	        	if (jump_state == 1){
    	        		chassis_state = 5;
    	        		break;
    	        	}
					if (staircase_state == 1){
						chassis_state = 6;
						break;
					}
    	        	error6900 = computeError(g_can_motors[19].raw_data.angle[0], 6900);
    	        	error2777 = computeError(g_can_motors[19].raw_data.angle[0], 2777);
    	        	//PID_Compute(&angleFilterPID, g_can_motors[19].angle_data.adj_ang,filtered_adj_ang , dt, 0);
    	        	//filtered_adj_ang = angleFilterPID.output;
    	        	//PID_Compute(&yawPID, target.yawAngle, filtered_adj_ang, dt, 0.01);

    	        	//PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);

    	        	//check_speed = leftWheel.speed - rightWheel.speed;
    	        	check_x = (leftWheel.angle +rightWheel.angle)/2.0f;
    	        	if (fabs(spin_speed)>0){
    	        		if (spin_toggle == 0){
    	        			spin_toggle = 1;
    	        		}
    	        		target.position = stateVar.x;
    	        		PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm,0.005,0);
    	        		//PID_Compute(&WheelspinPID, 0, leftWheel.speed - rightWheel.speed ,0.005,0);
    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;// + WheelspinPID.output;
    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;// - WheelspinPID.output;
    	        	}else{
    	        		if (spin_toggle == 1){
    	        			spin_toggle = 0;
    	        		}
    	        		if (fabs(error6900) < fabs(error2777)){
    	        			PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
    	        		}else{
    	        			PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
    	        		}
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
//    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
//    	        		leftForce -= (leftLegPos.length - 0.25f) * 100.0f;
//    	        	if(rightLegPos.length > 0.25f)
//    	        		rightForce -= (rightLegPos.length - 0.25f) * 100.0f;
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
    	        case 4: // robot floating
    	            target.position = stateVar.x; // Reset target position
    	            calculate_T_TP(0); // Not touching ground

    	            static TickType_t groundStateStartTime = 0; // Time when ground_state == 0 starts
    	            static int isGroundStateTimerActive = 0;   // Flag to track timer status

    	            PID_Compute(&LcushionPID, target.legLength, leftLegPos.length, dt, 0);
    	            PID_Compute(&RcushionPID, target.legLength, rightLegPos.length, dt, 0);
    	            PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);

    	            leftForce = LcushionPID.output + F_gravity + 60.0f;
    	            rightForce = RcushionPID.output + F_gravity + 60.0f;
    	            leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
    	            rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
    	            leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	            leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

    	            mf_set_tor[0] = 0;
    	            mf_set_tor[1] = 0;
    	            dm_set_tor[3] = leftJointTorque[0];
    	            dm_set_tor[0] = leftJointTorque[1];
    	            dm_set_tor[1] = -rightJointTorque[0];
    	            dm_set_tor[2] = -rightJointTorque[1];

    	            // Check ground state
    	            ground_state = ground_detect(leftForce, leftTp, stateVar.Ltheta, leftLegPos.length,
    	                                          rightForce, rightTp, stateVar.Rtheta, rightLegPos.length);

    	            if (ground_state == 0) { // If robot is not touching the ground
    	                if (!isGroundStateTimerActive) {
    	                    // Start the timer
    	                    groundStateStartTime = xTaskGetTickCount();
    	                    isGroundStateTimerActive = 1;
    	                } else if ((xTaskGetTickCount() - groundStateStartTime) * portTICK_PERIOD_MS >= 50) {
    	                    // If ground_state == 0 lasts for at least 0.1 seconds, change state
    	                    chassis_state = 3;
    	                    isGroundStateTimerActive = 0; // Reset the timer for the next check
    	                    break;
    	                }
    	            } else {
    	                // Reset the timer if ground_state != 0
    	                isGroundStateTimerActive = 0;
    	            }
    	            break;

    	        case 5: //jumping
    	        	if (jump_state ==1){
    	        		if (jump_state_l == 0 && jump_state_r == 0){
    	        			calculate_T_TP(1);
    	        			mf_set_tor[0] = -LlqrOutT * lqrTRatio;
    	        			mf_set_tor[1] = -RlqrOutT * lqrTRatio;
    	        			PID_Compute(&LlegLengthPID, target.min_legLength, leftLegPos.length,dt,0);
    	        			PID_Compute(&RlegLengthPID, target.min_legLength, rightLegPos.length,dt,0);
    	        			PID_Compute(&rollPID, target.rollAngle, INS.Roll,dt,0);
    	        			PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
    	        			leftForce = LlegLengthPID.output+ F_gravity-10.0f +rollPID.output;
    	        			rightForce = RlegLengthPID.output+ F_gravity-10.0f -rollPID.output;
    	        			leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
    	        			rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
    	        			leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	        			leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

    	        			dm_set_tor[3] = leftJointTorque[0];
    	        			dm_set_tor[0] = leftJointTorque[1];
    	        			dm_set_tor[1] = -rightJointTorque[0];
    	        			dm_set_tor[2] = -rightJointTorque[1];
    	        			if (leftLegPos.length < 0.16f){
    	        				jump_time_l++;
    	        			}
    	        			if (rightLegPos.length < 0.16f){
    	        				jump_time_r++;
    	        			}
    	        			if (jump_time_l>10 && jump_time_r>10){
    	        				jump_time_l = 0;
    	        				jump_time_r = 0;
    	        				jump_state_l = 1;
    	        				jump_state_r = 1;
    	        			}
    	        		}else if (jump_state_l == 1 && jump_state_r == 1){
        	        		calculate_T_TP(1);
        	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio;
        	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio;
        	        		PID_Compute(&LlegLengthPID, 0.6, leftLegPos.length,dt,0);
        	        		PID_Compute(&RlegLengthPID, 0.6, rightLegPos.length,dt,0);
        	        		PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
        	        		leftForce = LlegLengthPID.output+ F_gravity-10.0f;
        	        		rightForce = RlegLengthPID.output+ F_gravity-10.0f;
        	        		leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
        	        		rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
        	        		leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
        	        		leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

        	        		dm_set_tor[3] = leftJointTorque[0];
        	        		dm_set_tor[0] = leftJointTorque[1];
        	        		dm_set_tor[1] = -rightJointTorque[0];
        	        		dm_set_tor[2] = -rightJointTorque[1];
        	        		if (leftLegPos.length > 0.26f){
        	        			jump_time_l++;
        	        		}
        	        		if (rightLegPos.length > 0.26f){
        	        			jump_time_r++;
        	        		}
        	        		if (jump_time_l>10 && jump_time_r>10){
        	        			jump_time_l = 0;
        	        			jump_time_r = 0;
        	        			jump_state_l = 2;
        	        			jump_state_r = 2;
        	        		}
        	        	}else if (jump_state_l == 2 && jump_state_r == 2){
        	        		target.position = stateVar.x;
        	        		calculate_T_TP(0);
        	        		mf_set_tor[0] = 0;
        	        		mf_set_tor[1] = 0;
        	        		PID_Compute(&LlegLengthPID, 0.1, leftLegPos.length,dt,0);
        	        		PID_Compute(&RlegLengthPID, 0.1, rightLegPos.length,dt,0);
        	        		PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
        	        		leftForce = LlegLengthPID.output - 50.0f;
        	        		rightForce = RlegLengthPID.output - 50.0f;
        	        		leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
        	        		rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
        	        		leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
        	        		leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

        	        		dm_set_tor[3] = leftJointTorque[0];
        	        		dm_set_tor[0] = leftJointTorque[1];
        	        		dm_set_tor[1] = -rightJointTorque[0];
        	        		dm_set_tor[2] = -rightJointTorque[1];
        	        		if (leftLegPos.length < 0.2f){
        	        			jump_time_l++;
        	        		}
        	        		if (rightLegPos.length < 0.2f){
        	        			jump_time_r++;
        	        		}
        	        		if (jump_time_l>10 && jump_time_r>10){
        	        			jump_time_l = 0;
        	        			jump_time_r = 0;
        	        			jump_state_l = 3;
        	        			jump_state_r = 3;
        	        		}
        	        	}else if (jump_state_l == 3 && jump_state_r == 3){
        	        		jump_time_l = 0;
        	        		jump_time_r = 0;
        	        		jump_state_l = 0;
        	        		jump_state_r = 0;
        	        		jump_state = 0;
        	        		chassis_state = 4;
        	        	}
    	        	}

    	        	break;
    	        case 6:
    	        	calculate_T_TP(1);// 1 touching ground 0 not touching ground
    	        	if(staircase_detect != 0) //if robot not touching ground
    	        	{
    	        		chassis_state = 7;
    	        		break;
    	        	}
    	        	if (staircase_state == 0){
    	        		chassis_state = 3;
    	        		break;
    	        	}
    	        	error6900 = computeError(g_can_motors[19].raw_data.angle[0], 6900);
    	        	error2777 = computeError(g_can_motors[19].raw_data.angle[0], 2777);
    	        	//PID_Compute(&angleFilterPID, g_can_motors[19].angle_data.adj_ang,filtered_adj_ang , dt, 0);
    	        	//filtered_adj_ang = angleFilterPID.output;
    	        	//PID_Compute(&yawPID, target.yawAngle, filtered_adj_ang, dt, 0.01);

    	        	//PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);

    	        	//check_speed = leftWheel.speed - rightWheel.speed;
    	        	check_x = (leftWheel.angle +rightWheel.angle)/2.0f;
    	        	if (fabs(spin_speed)>0){
    	        		if (spin_toggle == 0){
    	        			spin_toggle = 1;
    	        		}
    	        		target.position = stateVar.x;
    	        		PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm,0.005,0);
    	        		//PID_Compute(&WheelspinPID, 0, leftWheel.speed - rightWheel.speed ,0.005,0);
    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;// + WheelspinPID.output;
    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;// - WheelspinPID.output;
    	        	}else{
    	        		if (spin_toggle == 1){
    	        			spin_toggle = 0;
    	        		}
    	        		if (fabs(error6900) < fabs(error2777)){
    	        			PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
    	        		}else{
    	        			PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
    	        		}
    	        		mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
    	        		mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
    	        	}


    	        	PID_Compute(&LlegLengthPID, 0.45, leftLegPos.length,dt,0);
    	        	PID_Compute(&RlegLengthPID, 0.45, rightLegPos.length,dt,0);
    	        	PID_Compute(&rollPID, target.rollAngle, INS.Roll,dt,0);
    	        	PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
    	        	//    	        	double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
    	        	//    	        	double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;

    	        	leftForce = LlegLengthPID.output + F_gravity +rollPID.output;
    	        	rightForce = RlegLengthPID.output + F_gravity -rollPID.output;
    	        	//    	        	if(leftLegPos.length > 0.25f) //保护腿部不能伸太长
    	        	//    	        		leftForce -= (leftLegPos.length - 0.25f) * 100.0f;
    	        	//    	        	if(rightLegPos.length > 0.25f)
    	        	//    	        		rightForce -= (rightLegPos.length - 0.25f) * 100.0f;
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

    	        	staircase_detect = ground_detect_staircase(leftForce,leftTp,stateVar.Ltheta,leftLegPos.length
    	        			,rightForce,rightTp,stateVar.Rtheta,rightLegPos.length);

    	        	break;
    	        case 7:
//    	        	dm_set_tor[3] = 0;
//    	        	dm_set_tor[0] = 0;
//    	        	dm_set_tor[1] = 0;
//    	        	dm_set_tor[2] = 0;
    	        	manual_set_legPos(0.4,0.25);
    	        	mf_set_tor[0] = 1;// + WheelspinPID.output;
    	        	mf_set_tor[1] = 1;
    	        	osDelay(250);
    	        	chassis_state = 8;
    	        	break;
    	        	//chassis_state = 8;
    	        case 8:
    	        	target.position = stateVar.x;
    	        	mf_set_tor[0] = 1;
    	        	mf_set_tor[1] = 1;
    	        	PID_Compute(&LlegLengthPID, 0.12, leftLegPos.length,dt,0);
    	        	PID_Compute(&RlegLengthPID, 0.12, rightLegPos.length,dt,0);
    	        	PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,dt,0.01);
    	        	leftForce = LlegLengthPID.output - 50.0f;
    	        	rightForce = RlegLengthPID.output - 50.0f;
    	        	leftTp = (legAnglePID.output + (leftLegPos.length));
    	        	rightTp = (legAnglePID.output + (rightLegPos.length));
    	        	leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
    	        	leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

    	        	dm_set_tor[3] = leftJointTorque[0];
    	        	dm_set_tor[0] = leftJointTorque[1];
    	        	dm_set_tor[1] = -rightJointTorque[0];
    	        	dm_set_tor[2] = -rightJointTorque[1];
    	        	if (leftLegPos.length < 0.15){
    	        		if(rightLegPos.length < 0.15){
    	        			chassis_state = 1;
    	        		}
    	        	}
    	        default: // Default case for invalid input
    	}
        vTaskDelay(5);
    }
}

