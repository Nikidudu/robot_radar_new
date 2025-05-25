#include "dm4310_drv.h"
#include <string.h>
#include "board_lib.h"
#include "robot_config.h"
#include "can_msg_processor.h"
#include "motor_control.h"
#include "motor_control_task.h"
#include "motor_config.h"
#include "INS_task.h"
#include "bsp_damiao.h"

dm_motor_t dm_pitch_motor;
dm_motor_t dm_yaw_motor;
float dm_yaw_set_vel;
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

// PID controllers
CascadePID gimbal_cpid_pit;
CascadePID gimbal_cpid_yaw;
PID gimbal_pid_yaw;
PID gimbal_pid_pitch;

// Debug variables
float yaw_error = 0;
float target_rad = 0;
float target_gimbal = 0.0f;
float dumbasss;
float debug1 = 0.3;
float debug2 = 6;
float test3 = 0;
float dm1 = 10;
float dm2 = 5;
float ex_pos = 0.0;
float debug3 = 0.0;

/************************* Main Control Task *************************/
void dm_motor_control_task(void *argument) {
    dm_set_tor[0] = 0.0f;
    dm_set_tor[1] = 0.0f;

    dm4310_motor_init();
    vTaskDelay(101);

    PID_Init(&gimbal_cpid_yaw.inner, 0.3, 0, 0.1, 0, 7);
    PID_Init(&gimbal_cpid_yaw.outer, 25, 0, 0.1, 0, 10);
    PID_Init(&gimbal_pid_yaw, 5, 0, 0, 0, 45);
    PID_Init(&gimbal_pid_pitch, 2.0, 0.0, 100.0, 0, 5);

    float dt = 0.003;
    TickType_t lastTick = xTaskGetTickCount();
    float prev_yaw = imu_heading.yaw;
    ex_pos = dm_yaw_motor.para.pos;

    while (1) {
        xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore, portMAX_DELAY);

        if(gimbal_ctrl_data.enabled == 1) {
            TickType_t currentTick = xTaskGetTickCount();
            dt = (currentTick - lastTick) / 1000.0f;
            lastTick = currentTick;

            target_rad = gimbal_ctrl_data.pitch;

            if (target_rad > 0.13f) {
                target_rad = 0.13f;
                gimbal_ctrl_data.pitch = 0.13f;
            } else if(target_rad < -0.70f) {
                target_rad = -0.70f;
                gimbal_ctrl_data.pitch = -0.70f;
            }

            PID_SingleCalc(&gimbal_pid_pitch, target_rad, INS.Pitch);

		    float turn_ang = imu_heading.yaw - prev_yaw;
//			float raw_pos = dm_yaw_motor.para.pos;
//			float turn_ang = raw_pos - prev_yaw;

		    while (turn_ang > PI) { turn_ang -= 2 * PI; }
		    while (turn_ang < -PI) { turn_ang += 2 * PI; }

		    dumbasss = turn_ang;
		    prev_yaw = imu_heading.yaw;
		    gimbal_ctrl_data.delta_yaw -= turn_ang;

		    if (gimbal_ctrl_data.delta_yaw > 1.5 * PI) { gimbal_ctrl_data.delta_yaw = 1.5 * PI; }
		    if (gimbal_ctrl_data.delta_yaw < -1.5 * PI) { gimbal_ctrl_data.delta_yaw = -1.5 * PI; }

            PID_SingleCalc(&gimbal_pid_yaw, 0, -gimbal_ctrl_data.delta_yaw);

            // TODO: use another logic for error checking (link to beeping sounds)
            if (dm_pitch_motor.para.state != 9 && dm_pitch_motor.para.disconnect_time > 100) {
                dm_pitch_motor.para.disconnect_time = 0;
                dm_pitch_motor.para.online = 0;
            } else {
                dm_pitch_motor.para.disconnect_time = 0;
                dm_pitch_motor.para.online = 1;
            }

            if (dm_pitch_motor.para.online == 1) {
                joint_motor_online = 1;
            } else {
                joint_motor_online = 0;
            }

            if (dm_yaw_motor.para.state != 9 && dm_yaw_motor.para.disconnect_time > 100) {
                dm_yaw_motor.para.disconnect_time = 0;
                dm_yaw_motor.para.online = 0;
            } else {
                dm_yaw_motor.para.disconnect_time = 0;
                dm_yaw_motor.para.online = 1;
            }

            if (dm_yaw_motor.para.online == 1) {
                joint_motor_online = 1;
            } else {
                joint_motor_online = 0;
            }

            // Disable pitch if kill switch is on
            if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
                dm_set_tor[0] = 0;
                dm_set_tor[1] = 0;
                dm_yaw_set_vel = 0;
                dm_set_tor[1] = 0;
            } else {
                dm_set_tor[0] = 0.4842f*imu_heading.pit - 2.3124f - gimbal_pid_pitch.output;
                dm_set_tor[0] *= 1.1;

                // PID output + yaw centering compensation + feedforward torque
                dm_set_tor[1] = FEEDFORWARD_CONST * dm_yaw_motor.para.vel;
                dm_yaw_set_vel = gimbal_pid_yaw.output +
                		chassis_ctrl_data.yaw * (YAW_SPINSPIN_CONSTANT/CHASSIS_SPINSPIN_MAX);
            }

            dm_pitch_motor.ctrl.tor_set = dm_set_tor[0];

            dm_yaw_motor.ctrl.vel_set = dm_yaw_set_vel;
            dm_yaw_motor.ctrl.tor_set = dm_set_tor[1];
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
        vTaskDelay(2);
    }
}

/************************* PID Functions *************************/
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->deadzone = 0;
    pid->errLpfRatio = 1;
}

void PID_SingleCalc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;
    if(ABS(reference-feedback) < pid->deadzone)
        pid->error = 0;
    else
        pid->error = reference - feedback;
    
    pid->error = pid->error * pid->errLpfRatio + pid->lastError * (1 - pid->errLpfRatio);
    pid->output = (pid->error - pid->lastError) * pid->kd;
    pid->output += pid->error * pid->kp;
    pid->integral += pid->error * pid->ki;
    LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral);
    pid->output += pid->integral;
    LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);
}

void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
    PID_SingleCalc(&pid->outer, angleRef, angleFdb);
    PID_SingleCalc(&pid->inner, pid->outer.output, speedFdb);
    pid->output = pid->inner.output;
}

void PID_Clear(PID *pid)
{
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_SetMaxOutput(PID *pid, float maxOut)
{
    pid->maxOutput = maxOut;
}

void PID_SetDeadzone(PID *pid, float deadzone)
{
    pid->deadzone = deadzone;
}

void PID_SetErrLpfRatio(PID *pid, float ratio)
{
    pid->errLpfRatio = ratio;
}

/************************* Helper Functions *************************/
float shortest_angular_difference(float current, float target) {
    float diff = fmodf(target - current + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    diff -= M_PI;
    return diff;
}

float dm_yaw_encoder_mod(float raw_angle) {
    float mapped_angle = fmod(P_MAX + raw_angle, 2*P_MAX/P_ROUNDS);
    return mapped_angle - P_MAX/P_ROUNDS;
}

void dmmapyawfbdata(dm_motor_t *yaw_motor) {
    g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang = dm_yaw_encoder_mod(yaw_motor->para.pos);
    g_can_motors[YAW_MOTOR_ID - 1].raw_data.torque = yaw_motor->para.tor;
}

void dmmappitchfbdata(dm_motor_t *pitch_motor) {
    g_pitch_motor.angle_data.adj_ang = pitch_motor->para.pos;
}
