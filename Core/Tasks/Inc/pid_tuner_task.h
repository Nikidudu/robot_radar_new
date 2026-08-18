/*
 * pid_tuner_task.h
 *
 * Bench-test harness for the yaw RPM/position PID (yaw_motor.rpm_pid).
 *
 * Problem this solves: STM32CubeMonitor can watch/log PID variables live,
 * but only while the ST-Link probe is physically attached, which is
 * impractical once the gimbal is off the bench. This module instead lets a
 * host script drive a step-response test and stream results over the
 * existing USB-CDC link (same cable already used for the SBC/aimbot link),
 * so no debugger needs to be plugged in to log a run.
 *
 * Flow:
 *   1. Host sends a pidTestCmdPacket (ID_PID_TEST_CMD) with candidate gains,
 *      a step size (radians), and a duration.
 *   2. pid_tuner_start() backs up the live gains, applies the candidates,
 *      and arms a one-shot position-error step of step_rad.
 *   3. Every gimbal-loop tick (2ms), gimbal_control_task calls
 *      pid_tuner_step(), which runs speed_pid() against the decaying step
 *      error (same "delta_yaw -= turn_ang" trick the normal single-PID-loop
 *      path uses) and buffers a telemetry sample.
 *   4. usb_task.c's parser task drains the sample ring buffer each
 *      iteration and streams pidTelemPacket / pidTestDonePacket to the host.
 *   5. After duration_ms, the original gains are restored automatically.
 *
 * Safety: this overrides yaw_motor's control output with an open-loop test
 * signal for the duration of the run, ignoring the normal joystick/aimbot
 * input. It only compiles in when PID_AUTOTUNE_ENABLE is defined in the
 * active robot_config - make sure that is NOT defined in any build that
 * will see a competition field. Bench/test builds only.
 */

#ifndef TASKS_INC_PID_TUNER_TASK_H_
#define TASKS_INC_PID_TUNER_TASK_H_

#include <stdint.h>
#include "typedefs.h"
#include "usb_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at boot (from USB_Firmware_Init or similar). */
void pid_tuner_init(void);

/* True while a test is running (gimbal_control_task uses this to decide
 * whether to run the tuner step instead of normal gimbal control). */
uint8_t pid_tuner_is_active(void);

/* Handle an inbound, already-CRC-validated test command. Starts a new run,
 * or is ignored if a run is already in progress. */
void pid_tuner_handle_cmd(const pidTestCmdPacket *cmd);

/* Call once per gimbal_control_task loop iteration (every GIMBAL_DELAY ms)
 * while pid_tuner_is_active() is true. turn_ang is the same "actual angle
 * moved since last tick" value the normal yaw loop computes from the IMU
 * (imu_heading.yaw - prev_yaw, wrapped to [-pi, pi]). Sets motor->output. */
void pid_tuner_step(motor_data_t *motor, float turn_ang);

/* Pop up to one buffered telemetry sample for transmission. Returns 1 and
 * fills *out if a sample was available, 0 otherwise. Non-blocking,
 * single-producer/single-consumer safe. */
uint8_t pid_tuner_pop_sample(pidTelemPacket *out);

/* Pop a pending "test finished" event, if any. Returns 1 and fills *out
 * once per completed run, 0 otherwise. */
uint8_t pid_tuner_pop_done(pidTestDonePacket *out);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_INC_PID_TUNER_TASK_H_ */
