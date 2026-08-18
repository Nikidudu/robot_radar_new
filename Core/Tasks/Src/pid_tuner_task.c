/*
 * pid_tuner_task.c
 *
 * See pid_tuner_task.h for the overall design. This file only implements
 * the state machine + sample buffering; USB framing/sending lives in
 * usb_task.c (it already owns CDC_Transmit_FS and the outbound packet
 * helpers), and the control-loop hook lives in gimbal_control_task.c.
 */

#include "board_lib.h"
#include "pid_tuner_task.h"
#include "motor_control.h"
#include "gimbal_control_task.h" /* extern motor_data_t yaw_motor; */

#ifdef PID_AUTOTUNE_ENABLE

/* ────────────────────────────────────────────────────────────────────────── */
/* Sample ring buffer (producer: gimbal_control_task via pid_tuner_step;
 * consumer: usb_task's parser task via pid_tuner_pop_sample). Single
 * producer / single consumer, same head/tail convention as usb_rb in
 * usb_task.c. */
/* ────────────────────────────────────────────────────────────────────────── */
#define PID_TUNER_RING_SIZE 128 /* power of 2 */

static volatile pidTelemPacket sample_ring[PID_TUNER_RING_SIZE];
static volatile uint32_t sample_head = 0;
static volatile uint32_t sample_tail = 0;

/* How often (in gimbal-loop ticks) to buffer a sample. At GIMBAL_DELAY=2ms
 * a decim of 2 gives ~250Hz telemetry, comfortably inside CDC full-speed
 * throughput while still resolving fast step responses. */
#define PID_TUNER_SAMPLE_DECIM 2

/* ────────────────────────────────────────────────────────────────────────── */
/* Test state */
/* ────────────────────────────────────────────────────────────────────────── */
typedef enum {
    TUNER_IDLE = 0,
    TUNER_RUNNING
} tuner_state_t;

static tuner_state_t state = TUNER_IDLE;
static pid_data_t saved_gains;      /* gains to restore once the run ends */
static pidTestCmdPacket active_cmd; /* gains/params under test, for the DONE echo */
static float test_error;            /* decaying position error, radians */
static uint32_t start_tick;
static uint32_t duration_ms;
static uint32_t sample_count;
static uint32_t tick_counter;

static volatile uint8_t done_pending = 0;
static pidTestDonePacket done_packet;

void pid_tuner_init(void)
{
    state = TUNER_IDLE;
    sample_head = 0;
    sample_tail = 0;
    done_pending = 0;
}

uint8_t pid_tuner_is_active(void)
{
    return (state == TUNER_RUNNING);
}

void pid_tuner_handle_cmd(const pidTestCmdPacket *cmd)
{
    if (state == TUNER_RUNNING) {
        /* Ignore new commands mid-run; host should wait for the DONE
         * packet before starting the next one. */
        return;
    }

    /* Back up whatever gains are currently live so they can be restored
     * exactly, whether or not they match the compiled-in defaults. */
    saved_gains = yaw_motor.rpm_pid;

    yaw_motor.rpm_pid.kp = cmd->kp;
    yaw_motor.rpm_pid.ki = cmd->ki;
    yaw_motor.rpm_pid.kd = cmd->kd;
    yaw_motor.rpm_pid.kff = cmd->kff;
    yaw_motor.rpm_pid.int_max = cmd->int_max;
    yaw_motor.rpm_pid.max_out = cmd->max_out;
    yaw_motor.rpm_pid.integral = 0;
    yaw_motor.rpm_pid.error[0] = 0;
    yaw_motor.rpm_pid.error[1] = 0;

    test_error = cmd->step_rad;
    duration_ms = cmd->duration_ms;
    start_tick = HAL_GetTick();
    sample_count = 0;
    tick_counter = 0;
    active_cmd = *cmd;

    state = TUNER_RUNNING;
}

static void pid_tuner_finish(motor_data_t *motor)
{
    /* Restore the gains that were live before this test. */
    motor->rpm_pid.kp = saved_gains.kp;
    motor->rpm_pid.ki = saved_gains.ki;
    motor->rpm_pid.kd = saved_gains.kd;
    motor->rpm_pid.kff = saved_gains.kff;
    motor->rpm_pid.int_max = saved_gains.int_max;
    motor->rpm_pid.max_out = saved_gains.max_out;
    motor->rpm_pid.integral = 0;
    motor->rpm_pid.error[0] = 0;
    motor->rpm_pid.error[1] = 0;
    motor->output = 0;

    done_packet.kp = active_cmd.kp;
    done_packet.ki = active_cmd.ki;
    done_packet.kd = active_cmd.kd;
    done_packet.kff = active_cmd.kff;
    done_packet.sample_count = sample_count;
    done_pending = 1;

    state = TUNER_IDLE;
}

void pid_tuner_step(motor_data_t *motor, float turn_ang)
{
    if (state != TUNER_RUNNING) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - start_tick) >= duration_ms) {
        pid_tuner_finish(motor);
        return;
    }

    test_error -= turn_ang;
    speed_pid(test_error, 0, &motor->rpm_pid);
    motor->output = motor->rpm_pid.output;

    tick_counter++;
    if ((tick_counter % PID_TUNER_SAMPLE_DECIM) == 0) {
        uint32_t next_head = (sample_head + 1) & (PID_TUNER_RING_SIZE - 1);
        if (next_head != sample_tail) { /* drop sample if the ring is full rather than overwrite */
            sample_ring[sample_head].t_ms = now - start_tick;
            sample_ring[sample_head].error = test_error;
            sample_ring[sample_head].output = motor->rpm_pid.output;
            sample_ring[sample_head].integral = motor->rpm_pid.integral;
            sample_head = next_head;
            sample_count++;
        }
    }
}

uint8_t pid_tuner_pop_sample(pidTelemPacket *out)
{
    if (sample_tail == sample_head) {
        return 0;
    }
    *out = sample_ring[sample_tail];
    sample_tail = (sample_tail + 1) & (PID_TUNER_RING_SIZE - 1);
    return 1;
}

uint8_t pid_tuner_pop_done(pidTestDonePacket *out)
{
    if (!done_pending) {
        return 0;
    }
    *out = done_packet;
    done_pending = 0;
    return 1;
}

#else /* !PID_AUTOTUNE_ENABLE: stub everything out so callers don't need
       * #ifdefs sprinkled through gimbal_control_task.c / usb_task.c. */

void pid_tuner_init(void) {}
uint8_t pid_tuner_is_active(void) { return 0; }
void pid_tuner_handle_cmd(const pidTestCmdPacket *cmd) { (void)cmd; }
void pid_tuner_step(motor_data_t *motor, float turn_ang) { (void)motor; (void)turn_ang; }
uint8_t pid_tuner_pop_sample(pidTelemPacket *out) { (void)out; return 0; }
uint8_t pid_tuner_pop_done(pidTestDonePacket *out) { (void)out; return 0; }

#endif /* PID_AUTOTUNE_ENABLE */
