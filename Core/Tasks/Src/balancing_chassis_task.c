/*
 * balancing_chassis_task.c
 *
 *  Created on: Dec 10, 2024
 *      Author: YI MING
 */
#include <math.h>
#include "board_lib.h"
#include "robot_config.h"
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
extern motor_t motor[num];
extern remote_cmd_t g_remote_cmd;
extern motor_data_t g_can_motors[24];
extern float dm_set_tor[4];
extern float mf_set_tor[2];
PID yawPID, rollPID;
PID legAnglePID, LlegLengthPID, RlegLengthPID;
PID spinPID;
PID LcushionPID, RcushionPID;
int chassis_state = 0;
int ground_state = 0;
float BODY_MASS = 20.0f;
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
int robot_ready = 0; // 1 ready, 0 not ready
PID manual_left_F, manual_left_Tp, manual_right_F, manual_right_Tp;
float kRatio[2][6] = {{1.0f, 0.8f, 1.0f, 1.0f, 1.0f, 1.0f},
                      {1.0f, 0.8f, 1.0f, 1.0f, 1.0f, 1.0f}};
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
const float alpha = 0.5f; // Smoothing factor
int spin_toggle = 0;
int jump_state = 0;
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
uint8_t staircase_detect = 0; // if 1 means robot hit the staircase edge
float threshold_variation;
float filtered_angle = 0.0f;
const float angle_alpha = 0.15f; // Smoothing factor (0.0-1.0), lower = more filtering
float filtered_LdTheta = 0.0f;
float filtered_RdTheta = 0.0f;
const float dTheta_alpha = 0.5f; // Smoothing factor for angular velocity values

// Define state transition matrix (F), covariance (P), process noise (Q), and measurement noise (R)
float angleKF_F[1] = {1.0f};    // Simple model where next state is the same as the current state
float angleKF_P[1] = {1.0f};    // Initial covariance
float angleKF_Q[1] = {0.01f};   // Process noise (small, since angle is relatively stable)
float angleKF_R[1] = {5.0f};    // Measurement noise (adjust based on sensor noise level)
const float angleKF_H[1] = {1.0f}; // Measurement model
float forward = 0.0f;
float backward = 0.0f;
float left = 0.0f;
float right = 0.0f;
extern uint8_t control_mode;

void Ctrl_Init()
{
    // Robot main PID initialization
    PID_Init(&LlegLengthPID, 500, 0.0, 150.0, -200.0, 200.0);
    PID_Init(&RlegLengthPID, 500, 0.0, 150.0, -200.0, 200.0);
    PID_Init(&LcushionPID, 800, 0.0, 150.0, -200.0, 200.0);
    PID_Init(&RcushionPID, 800, 0.0, 150.0, -200.0, 200.0);
    PID_Init(&legAnglePID, 30, 0.1, 1, -50.0, 50.0);
    PID_Init(&rollPID, 500, 0.0, 2.0, -200.0, 200.0);
    PID_Init(&yawPID, 0.004, 0.0001, 0.0026, -5.0, 5.0);
    PID_Init(&spinPID, 3.0, 0.0, 0.1, -2.0, 2.0);
}

#define MAX_ANGLE 8000
#define HALF_RANGE (MAX_ANGLE / 2)
int computeError(int current, int target) {
    return ((current - target + HALF_RANGE) % MAX_ANGLE) - HALF_RANGE;
}

void Ctrl_TargetUpdateTask()
{


    TickType_t xLastWakeTime = xTaskGetTickCount();
    float speedSlopeStep = 0.4f;
    float speedCmdSlope = 0.01f;

    while (1)
    {
    	if (g_remote_cmd.keyboard_keys & KEY_OFFSET_W) {
    		forward = KEYBD_MAX_SPD;
    	}else if (g_remote_cmd.keyboard_keys & KEY_OFFSET_S) {
    		forward = -KEYBD_MAX_SPD;
    	}else{
    		forward = 0;
    	}

    	if (g_remote_cmd.keyboard_keys & KEY_OFFSET_A) {
    		left = KEYBD_MAX_SPD;
    	}
    	if (g_remote_cmd.keyboard_keys & KEY_OFFSET_D) {
    		right = -KEYBD_MAX_SPD;
    	}
        float desiredSpeedCmd;
        if (fabs(error6900) < fabs(error2777)){
        	if (control_mode == KEYBOARD_CTRL_MODE){
        		desiredSpeedCmd = forward * 2.5f;
        	}else{
        		desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * 2.5f;
        	}
        }else{
        	if (control_mode == KEYBOARD_CTRL_MODE){
        		desiredSpeedCmd = forward * -2.5f;
        	}else{
        		desiredSpeedCmd = ((float)g_remote_cmd.left_y / 660) * -2.5f;
        	}
        }

        if (desiredSpeedCmd == 0.0f) {
            target.speedCmd = 0.0f;
        }
        else if ((desiredSpeedCmd > 0 && target.speedCmd < 0) ||
                 (desiredSpeedCmd < 0 && target.speedCmd > 0)) {
            target.speedCmd = 0.0f;
        }
        else if (fabs(desiredSpeedCmd - target.speedCmd) < speedCmdSlope) {
            target.speedCmd = desiredSpeedCmd;
        }
        else {
            if (desiredSpeedCmd > target.speedCmd)
                target.speedCmd += speedCmdSlope;
            else
                target.speedCmd -= speedCmdSlope;
        }

        if ((float)g_remote_cmd.side_dial > 0){
            spin_speed = ((float)g_remote_cmd.side_dial / 660) * 90.0f;
        }else{
            spin_speed = 0;
        }
        
        if (chassis_state == 3 || chassis_state == 6){
            if (g_remote_cmd.side_dial == -660 && staircase_toggle != -660) {
                staircase_state = !staircase_state;
                staircase_toggle = -660;
            } else if (g_remote_cmd.side_dial > -360) {
                staircase_toggle = 0;
            }
        }

        float legLength = (leftLegPos.length + rightLegPos.length) / 2;
        speedSlopeStep = -(legLength - 0.15f) * 0.03f + 1.0f;

        if (fabs(target.speedCmd - target.speed) < speedSlopeStep) {
            target.speed = target.speedCmd;
        }
        else {
            if (target.speedCmd - target.speed > 0)
                target.speed += speedSlopeStep;
            else
                target.speed -= speedSlopeStep;
        }

        target.position += target.speed * 0.005f;
        if (target.position - stateVar.x > 0.5f)
            target.position = stateVar.x + 0.5f;
        else if (target.position - stateVar.x < -0.5f)
            target.position = stateVar.x - 0.5f;

        if (target.speed - stateVar.dx > 1.5f)
            target.speed = stateVar.dx + 1.5f;
        else if (target.speed - stateVar.dx < -1.5f)
            target.speed = stateVar.dx - 1.5f;
        target.legLength = 0.19f + ((float)g_remote_cmd.left_x / 660)*0.07f;
        
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

int ground_detect(float LF, float LTP, float Ltheta, float LL0, float RF, float RTP, float Rtheta, float RL0) {
    threshold_variation = fabs(sin((stateVar.Ltheta + stateVar.Rtheta)/2)*50);
    LFN = LF * arm_cos_f32(Ltheta) + LTP * arm_sin_f32(Ltheta) / LL0;
    RFN = RF * arm_cos_f32(Rtheta) + RTP * arm_sin_f32(Rtheta) / RL0;

    LFN_filtered = alpha * LFN + (1.0f - alpha) * LFN_filtered;
    RFN_filtered = alpha * RFN + (1.0f - alpha) * RFN_filtered;

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
    LFN = LF * arm_cos_f32(Ltheta) + LTP * arm_sin_f32(Ltheta) / LL0;
    RFN = RF * arm_cos_f32(Rtheta) + RTP * arm_sin_f32(Rtheta) / RL0;

    LFN_filtered = alpha * LFN + (1.0f - alpha) * LFN_filtered;
    RFN_filtered = alpha * RFN + (1.0f - alpha) * RFN_filtered;

    Average_FN = (LFN_filtered + RFN_filtered)/2.0f;

    if ((INS.Accel[0] < (-10.0f) && target.speedCmd > 0.1f) || (INS.Accel[0] > (10.0f) && target.speedCmd < -0.1f))  {
        return 1;
    } else {
        return 0;
    }
}

int robot_check() {
    return 1;
}

void state_update() {
    stateVar.phi = INS.Pitch;
    stateVar.dPhi = -INS.Gyro[1];
    stateVar.x = filtered_x;
    stateVar.dx = filtered_v;
    stateVar.Ltheta = leftLegPos.angle - M_PI_2 - INS.Pitch;
    
    // Apply low-pass filter to left leg angular velocity
    float raw_LdTheta = leftLegPos.dAngle - (-INS.Gyro[1]);
    filtered_LdTheta = dTheta_alpha * raw_LdTheta + (1.0f - dTheta_alpha) * filtered_LdTheta;
    stateVar.LdTheta = filtered_LdTheta;
    
    stateVar.Rtheta = rightLegPos.angle - M_PI_2 - INS.Pitch;
    
    // Apply low-pass filter to right leg angular velocity
    float raw_RdTheta = rightLegPos.dAngle - (-INS.Gyro[1]);
    filtered_RdTheta = dTheta_alpha * raw_RdTheta + (1.0f - dTheta_alpha) * filtered_RdTheta;
    stateVar.RdTheta = filtered_RdTheta;
    
    stateVar.legLength = (leftLegPos.length + rightLegPos.length) / 2;
    stateVar.dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;
}

void kill_chassis() {
    dm_set_tor[3] = 0;
    dm_set_tor[0] = 0;
    dm_set_tor[1] = 0;
    dm_set_tor[2] = 0;
    mf_set_tor[0] = 0;
    mf_set_tor[1] = 0;
    target.speed = 0.0f;
    target.position = stateVar.x;
}

void manual_set_PidInit() {
    PID_Init(&manual_left_F, 400, 0, 5, -70, 70);
    PID_Init(&manual_left_Tp, 20.0, 0, 1.0, -50, 50);
    PID_Init(&manual_right_F, 400, 0, 5, -70, 70);
    PID_Init(&manual_right_Tp, 20.0, 0, 1.0, -50, 50);
}

void manual_set_legPos(float angle, float legLength) {
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
    
    leg_conv(left_F_control, left_Tp_control, leftJoint[0].angle, leftJoint[1].angle, leftTorque);
    leg_conv(right_F_control, right_Tp_control, rightJoint[0].angle, rightJoint[1].angle, rightTorque);
    
    if (robot_ready == 1) {
        dm_set_tor[3] = leftTorque[0];
        dm_set_tor[0] = leftTorque[1];
        dm_set_tor[1] = rightTorque[0];
        dm_set_tor[2] = rightTorque[1];
    } else {
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

void gimbal_auto_front(){ // Find shortest distance to align robot gimbal and body
    // Apply low-pass filter to raw angle data
    filtered_angle = angle_alpha * g_can_motors[19].raw_data.angle[0] + (1.0f - angle_alpha) * filtered_angle;
    
    // Use filtered angle for direction toggle logic
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
    float leftForce;
    float rightForce;
    float leftTp;
    float rightTp;
    float leftJointTorque[2]={0};
    float rightJointTorque[2]={0};
    
    // Set initial target values
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
        
        if (g_remote_cmd.right_switch == 1) {
            chassis_state = 0;
        }
        if (g_remote_cmd.right_switch == 3) {
            chassis_state = 1;
        }
        
        gimbal_auto_front(); // Align gimbal 0 or 180
        F_gravity = fabs(cos((stateVar.Ltheta + stateVar.Rtheta)/2)*8.0f*9.81f);
        
        switch (chassis_state) {
            case 0: // Robot die
                kill_chassis();
                jump_state = 0;
                staircase_state = 0;
                target.position = stateVar.x; // Reset target pos
                ground_state = 0; // On ground
                if (robot_ready == 1) {
                    chassis_state = 1;
                }
                break;

            case 1: // Leg positioning
                manual_set_legPos(0,0.12);
                if ((leftLegPos.angle >1.3 && leftLegPos.angle < 1.7) &&
                    (leftLegPos.length >0.1 && leftLegPos.length < 0.15) &&
                    (rightLegPos.angle >1.3 && rightLegPos.angle < 1.7) &&
                    (rightLegPos.length >0.1 && rightLegPos.length < 0.15)) {
                    if (g_remote_cmd.right_switch == 2) {
                        chassis_state = 2;
                        break;
                    }
                }
                mf_set_tor[0] = ((float)g_remote_cmd.left_y/660.0f)*5.0f + ((float)g_remote_cmd.left_x/660.0f)*5.0f;
                mf_set_tor[1] = ((float)g_remote_cmd.left_y/660.0f)*5.0f - ((float)g_remote_cmd.left_x/660.0f)*5.0f;
                break;

            case 2: // Standing with min leglength
                target.speed = 0.0f;
                staircase_state = 0;
                target.position = stateVar.x;
                calculate_T_TP(1); // 1 touching ground, 0 not touching ground
                error6900 = computeError(filtered_angle, 6900);
                error2777 = computeError(filtered_angle, 2777);

                if (fabs(error6900) < fabs(error2777)) {
                    PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
                } else {
                    PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
                }
                mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
                mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;

                PID_Compute(&LlegLengthPID, target.min_legLength, leftLegPos.length, dt, 0);
                PID_Compute(&RlegLengthPID, target.min_legLength, rightLegPos.length, dt, 0);
                PID_Compute(&rollPID, target.rollAngle, INS.Roll, dt, 0);
                PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);

                leftForce = LlegLengthPID.output + F_gravity + rollPID.output;
                rightForce = RlegLengthPID.output + F_gravity - rollPID.output;
                leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
                rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));

                leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
                leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

                dm_set_tor[3] = leftJointTorque[0];
                dm_set_tor[0] = leftJointTorque[1];
                dm_set_tor[1] = -rightJointTorque[0];
                dm_set_tor[2] = -rightJointTorque[1];

                static TickType_t steadyStateStartTime = 0;
                static int isSteadyStateTimerActive = 0;

                int isSteadyState = (fabs(stateVar.phi) < 0.12) &&
                                  (fabs(stateVar.Ltheta) < 0.12) &&
                                  (fabs(stateVar.Rtheta) < 0.12);

                if (isSteadyState) {
                    if (!isSteadyStateTimerActive) {
                        steadyStateStartTime = xTaskGetTickCount();
                        isSteadyStateTimerActive = 1;
                    } else if ((xTaskGetTickCount() - steadyStateStartTime) * portTICK_PERIOD_MS >= 300) {
                        chassis_state = 3;
                        break;
                    }
                } else {
                    isSteadyStateTimerActive = 0;
                }
                break;
                
            case 3: // Robot standing with target leglength
                staircase_detect = 0;
                calculate_T_TP(1); // 1 touching ground, 0 not touching ground
                if (ground_state != 0) { // If robot not touching ground
                    chassis_state = 4;
                    break;
                }
                if (jump_state == 1) {
                    chassis_state = 5;
                    break;
                }
                if (staircase_state == 1) {
                    chassis_state = 6;
                    break;
                }
                error6900 = computeError(filtered_angle, 6900);
                error2777 = computeError(filtered_angle, 2777);

                if (fabs(spin_speed) > 0) {
                    if (spin_toggle == 0) {
                        spin_toggle = 1;
                    }
                    target.position = stateVar.x;
                    PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm, 0.005, 0);
                    mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;
                    mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;
                } else {
                    if (spin_toggle == 1) {
                        spin_toggle = 0;
                    }
                    if (fabs(error6900) < fabs(error2777)) {
                        PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
                    } else {
                        PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
                    }
                    mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
                    mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
                }

                PID_Compute(&LlegLengthPID, target.legLength, leftLegPos.length, dt, 0);
                PID_Compute(&RlegLengthPID, target.legLength, rightLegPos.length, dt, 0);
                PID_Compute(&rollPID, target.rollAngle, INS.Roll, dt, 0);
                PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);

                leftForce = LlegLengthPID.output + F_gravity + rollPID.output;
                rightForce = RlegLengthPID.output + F_gravity - rollPID.output;
                leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
                rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));

                leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
                leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

                dm_set_tor[3] = leftJointTorque[0];
                dm_set_tor[0] = leftJointTorque[1];
                dm_set_tor[1] = -rightJointTorque[0];
                dm_set_tor[2] = -rightJointTorque[1];

                ground_state = ground_detect(leftForce, leftTp, stateVar.Ltheta, leftLegPos.length,
                                            rightForce, rightTp, stateVar.Rtheta, rightLegPos.length);
                break;
                
            case 4: // Robot floating
                target.position = stateVar.x; // Reset target position
                calculate_T_TP(0); // Not touching ground

                static TickType_t groundStateStartTime = 0;
                static int isGroundStateTimerActive = 0;

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
                        groundStateStartTime = xTaskGetTickCount();
                        isGroundStateTimerActive = 1;
                    } else if ((xTaskGetTickCount() - groundStateStartTime) * portTICK_PERIOD_MS >= 50) {
                        chassis_state = 3;
                        isGroundStateTimerActive = 0;
                        break;
                    }
                } else {
                    isGroundStateTimerActive = 0;
                }
                break;

            case 5: // Jumping
                if (jump_state == 1) {
                    if (jump_state_l == 0 && jump_state_r == 0) {
                        calculate_T_TP(1);
                        mf_set_tor[0] = -LlqrOutT * lqrTRatio;
                        mf_set_tor[1] = -RlqrOutT * lqrTRatio;
                        PID_Compute(&LlegLengthPID, target.min_legLength, leftLegPos.length, dt, 0);
                        PID_Compute(&RlegLengthPID, target.min_legLength, rightLegPos.length, dt, 0);
                        PID_Compute(&rollPID, target.rollAngle, INS.Roll, dt, 0);
                        PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);
                        leftForce = LlegLengthPID.output + F_gravity - 10.0f + rollPID.output;
                        rightForce = RlegLengthPID.output + F_gravity - 10.0f - rollPID.output;
                        leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
                        rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
                        leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
                        leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

                        dm_set_tor[3] = leftJointTorque[0];
                        dm_set_tor[0] = leftJointTorque[1];
                        dm_set_tor[1] = -rightJointTorque[0];
                        dm_set_tor[2] = -rightJointTorque[1];
                        if (leftLegPos.length < 0.16f) {
                            jump_time_l++;
                        }
                        if (rightLegPos.length < 0.16f) {
                            jump_time_r++;
                        }
                        if (jump_time_l > 10 && jump_time_r > 10) {
                            jump_time_l = 0;
                            jump_time_r = 0;
                            jump_state_l = 1;
                            jump_state_r = 1;
                        }
                    } else if (jump_state_l == 1 && jump_state_r == 1) {
                        calculate_T_TP(1);
                        mf_set_tor[0] = -LlqrOutT * lqrTRatio;
                        mf_set_tor[1] = -RlqrOutT * lqrTRatio;
                        PID_Compute(&LlegLengthPID, 0.6, leftLegPos.length, dt, 0);
                        PID_Compute(&RlegLengthPID, 0.6, rightLegPos.length, dt, 0);
                        PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);
                        leftForce = LlegLengthPID.output + F_gravity - 10.0f;
                        rightForce = RlegLengthPID.output + F_gravity - 10.0f;
                        leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
                        rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));
                        leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
                        leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

                        dm_set_tor[3] = leftJointTorque[0];
                        dm_set_tor[0] = leftJointTorque[1];
                        dm_set_tor[1] = -rightJointTorque[0];
                        dm_set_tor[2] = -rightJointTorque[1];
                        if (leftLegPos.length > 0.26f) {
                            jump_time_l++;
                        }
                        if (rightLegPos.length > 0.26f) {
                            jump_time_r++;
                        }
                        if (jump_time_l > 10 && jump_time_r > 10) {
                            jump_time_l = 0;
                            jump_time_r = 0;
                            jump_state_l = 2;
                            jump_state_r = 2;
                        }
                    } else if (jump_state_l == 2 && jump_state_r == 2) {
                        target.position = stateVar.x;
                        calculate_T_TP(0);
                        mf_set_tor[0] = 0;
                        mf_set_tor[1] = 0;
                        PID_Compute(&LlegLengthPID, 0.1, leftLegPos.length, dt, 0);
                        PID_Compute(&RlegLengthPID, 0.1, rightLegPos.length, dt, 0);
                        PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);
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
                        if (leftLegPos.length < 0.2f) {
                            jump_time_l++;
                        }
                        if (rightLegPos.length < 0.2f) {
                            jump_time_r++;
                        }
                        if (jump_time_l > 10 && jump_time_r > 10) {
                            jump_time_l = 0;
                            jump_time_r = 0;
                            jump_state_l = 3;
                            jump_state_r = 3;
                        }
                    } else if (jump_state_l == 3 && jump_state_r == 3) {
                        jump_time_l = 0;
                        jump_time_r = 0;
                        jump_state_l = 0;
                        jump_state_r = 0;
                        jump_state = 0;
                        chassis_state = 4;
                    }
                }
                break;
                
            case 6: // Staircase mode
                calculate_T_TP(1);
                if (staircase_detect != 0) {
                    chassis_state = 7;
                    break;
                }
                if (staircase_state == 0) {
                    chassis_state = 3;
                    break;
                }
                error6900 = computeError(filtered_angle, 6900);
                error2777 = computeError(filtered_angle, 2777);

                if (fabs(spin_speed) > 0) {
                    if (spin_toggle == 0) {
                        spin_toggle = 1;
                    }
                    target.position = stateVar.x;
                    PID_Compute(&spinPID, spin_speed, g_can_motors[19].raw_data.rpm, 0.005, 0);
                    mf_set_tor[0] = -LlqrOutT * lqrTRatio + spinPID.output;
                    mf_set_tor[1] = -RlqrOutT * lqrTRatio - spinPID.output;
                } else {
                    if (spin_toggle == 1) {
                        spin_toggle = 0;
                    }
                    if (fabs(error6900) < fabs(error2777)) {
                        PID_Compute(&yawPID, target.yawAngle, error6900, dt, 0);
                    } else {
                        PID_Compute(&yawPID, target.yawAngle, error2777, dt, 0);
                    }
                    mf_set_tor[0] = -LlqrOutT * lqrTRatio + yawPID.output;
                    mf_set_tor[1] = -RlqrOutT * lqrTRatio - yawPID.output;
                }

                PID_Compute(&LlegLengthPID, 0.42, leftLegPos.length, dt, 0);
                PID_Compute(&RlegLengthPID, 0.42, rightLegPos.length, dt, 0);
                PID_Compute(&rollPID, target.rollAngle, INS.Roll, dt, 0);
                PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);

                leftForce = LlegLengthPID.output + F_gravity + rollPID.output;
                rightForce = RlegLengthPID.output + F_gravity - rollPID.output;
                leftTp = -LlqrOutTp * lqrTpRatio + (legAnglePID.output + (leftLegPos.length));
                rightTp = -RlqrOutTp * lqrTpRatio - (legAnglePID.output + (rightLegPos.length));

                leg_conv(leftForce, leftTp, leftJoint[0].angle, leftJoint[1].angle, leftJointTorque);
                leg_conv(rightForce, rightTp, rightJoint[0].angle, rightJoint[1].angle, rightJointTorque);

                dm_set_tor[3] = leftJointTorque[0];
                dm_set_tor[0] = leftJointTorque[1];
                dm_set_tor[1] = -rightJointTorque[0];
                dm_set_tor[2] = -rightJointTorque[1];

                staircase_detect = ground_detect_staircase(leftForce, leftTp, stateVar.Ltheta, leftLegPos.length,
                                                         rightForce, rightTp, stateVar.Rtheta, rightLegPos.length);
                break;
                
            case 7: // Staircase detect - crossing edge
                if (fabs(error6900) < fabs(error2777)) {
                    manual_set_legPos(0.35, 0.2);
                    mf_set_tor[0] = 1;
                    mf_set_tor[1] = 1;
                    if (stateVar.Ltheta > 0.35 && stateVar.Rtheta > 0.35) {
                        chassis_state = 8;
                    }
                } else {
                    manual_set_legPos(-0.35, 0.2);
                    mf_set_tor[0] = -1;
                    mf_set_tor[1] = -1;
                    if (stateVar.Ltheta < -0.35 && stateVar.Rtheta < -0.35) {
                        chassis_state = 8;
                    }
                }
                break;
                
            case 8: // Prepare to return to normal stance
                target.position = stateVar.x;

                PID_Compute(&LlegLengthPID, 0.12, leftLegPos.length, dt, 0);
                PID_Compute(&RlegLengthPID, 0.12, rightLegPos.length, dt, 0);
                PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, dt, 0.01);
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
                
                if (leftLegPos.length < 0.15 && rightLegPos.length < 0.15) {
                    chassis_state = 1;
                }
                break;
                
            default:
                break;
        }
        vTaskDelay(5);
    }
}

