#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "bsp_damiao.h"

#ifdef __cplusplus
extern "C" {
#endif

// Main task function
void dm_motor_control_task(void *argument);

#define FEEDFORWARD_CONST 0.3
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#endif

// Type definitions for PID control
typedef struct _PID {
    float kp, ki, kd;
    float error, lastError;
    float integral, maxIntegral;
    float output, maxOutput;
    float deadzone;
    float errLpfRatio;
} PID;

typedef struct _CascadePID {
    PID inner;
    PID outer;
    float output;
} CascadePID;

// PID control functions
void PID_Init(PID *pid, float p, float i, float d, float maxSum, float maxOut);
void PID_SingleCalc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);
void PID_Clear(PID *pid);
void PID_SetMaxOutput(PID *pid, float maxOut);
void PID_SetDeadzone(PID *pid, float deadzone);
void PID_SetErrLpfRatio(PID *pid, float ratio);

// Helper functions
float shortest_angular_difference(float current, float target);
float dm_yaw_encoder_mod(float raw_angle);

// Motor data mapping functions
void dmmapyawfbdata(motor_t *yaw_motor);
void dmmappitchfbdata(motor_t *pitch_motor);

#ifdef __cplusplus
}
#endif

#endif /* __DM4310_DRV_H__ */
