/*
 * usb_config_task.h
 *
 *  Created on: Dec 20, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_USB_TASK_H_
#define TASKS_INC_USB_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ────────────────────────────────────────────────────────────────────────── */
/* Protocol Constants */
/* ────────────────────────────────────────────────────────────────────────── */
#define USB_MAGIC_BYTE      0x7F
#define USB_MAX_PAYLOAD_SIZE 256

/* ────────────────────────────────────────────────────────────────────────── */
/* Protocol IDs for NUS25 (integrated from BRoCo) */
/* ────────────────────────────────────────────────────────────────────────── */
#define ID_GIMBAL_JOINTS            3
#define ID_DUMMY                    4
#define ID_CHASSIS_SPEED            6
#define ID_LEFT_TRIGGER             8
#define ID_CHASSIS_SPIN             10
#define ID_COMPETITION_STATUS       11
#define ID_GIMBAL_COMMAND           12
#define ID_FIRING_COMMAND           13
#define ID_CV_DETECTED              14
#define ID_SURVEIL_COMMAND          15
#define ID_AIM_COMMAND              16
#define ID_IS_NAVIGATING            17
#define ID_OCCUPATION_STATUS        19
#define ID_WIN_STATUS               20
#define ID_STATE                   21   /* State machine number only (0-20) */

/* PID bench-tuning packets (see pid_tuner_task.h). Host -> device: ID_PID_TEST_CMD.
 * Device -> host: ID_PID_TELEM (streamed during a run), ID_PID_TEST_DONE (end of run). */
#define ID_PID_TEST_CMD             30
#define ID_PID_TELEM                31
#define ID_PID_TEST_DONE            32

/* ────────────────────────────────────────────────────────────────────────── */
/* Protocol Macros & CRC (integrated from BRoCo) */
/* ────────────────────────────────────────────────────────────────────────── */

static inline uint16_t usb_protocol_crc16(const uint8_t *data, uint16_t size) {
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (size--){
        x = crc >> 8 ^ *data++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

#define RELIABLE_PACKET(NAME, PACKET_DEF) typedef struct __attribute__((packed)) NAME { PACKET_DEF uint16_t crc; } NAME;
#define RELIABLE_IDENTIFIABLE_PACKET(NAME, PACKET_DEF) typedef struct __attribute__((packed)) NAME { PACKET_DEF uint16_t id; uint16_t crc; } NAME;

#define MAKE_RELIABLE(PACKET) (PACKET).crc = usb_protocol_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)
#define IS_RELIABLE(PACKET) (PACKET).crc == usb_protocol_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)

/* ────────────────────────────────────────────────────────────────────────── */
/* Packet Definitions (integrated from BRoCo) */
/* ────────────────────────────────────────────────────────────────────────── */

RELIABLE_PACKET(gimbalJointsPacket,
    float yaw_angle;
    float pitch_angle;
)

RELIABLE_IDENTIFIABLE_PACKET(dummyPacket,
    int num1;
    int num2;
    int num3;
)

RELIABLE_PACKET(chassisSpeedCommandPacket,
    float V_horz;
    float V_lat;
    float V_yaw;
)

RELIABLE_PACKET(competitionStatusPacket,
    uint16_t game_progress;
    uint16_t time_left;
    uint16_t robot_id;
    uint16_t current_hp;
    uint16_t red_hero_hp;
    uint16_t red_standard_hp;
    uint16_t red_sentry_hp;
    uint16_t blue_hero_hp;
    uint16_t blue_standard_hp;
    uint16_t blue_sentry_hp;
)

/* Simple packet: just the state number (0-20) */
RELIABLE_PACKET(statePacket,
    uint8_t state;
)

RELIABLE_PACKET(cvGimbalCommandPacket,
    float pitch;      /* bytes 0-3: pitch_error (rad) */
    float yaw;        /* bytes 4-7: yaw_error (rad) */
    uint8_t tracking; /* byte 8: 1=armor detected, 0=not */
)

RELIABLE_PACKET(firingCommandPacket,
    uint8_t fire_state; /* byte 0: 1=fire, 0=don't fire */
)

RELIABLE_PACKET(surveilCommandPacket,
    bool surveillance_state;
)

RELIABLE_PACKET(aimCommandPacket,
    bool aiming_state;
)

RELIABLE_PACKET(isNavigatingPacket,
    bool navigating_state;
)

/* Host -> device: request a step-response bench test on yaw_motor.rpm_pid.
 * Applies the given gains, injects a one-shot step of step_rad radians into
 * the loop's position error, streams telemetry for duration_ms, then
 * restores the gains that were active before the test. Gated behind
 * PID_AUTOTUNE_ENABLE - see pid_tuner_task.h. */
RELIABLE_PACKET(pidTestCmdPacket,
    float kp;
    float ki;
    float kd;
    float kff;
    float int_max;
    float max_out;
    float step_rad;      /* size of the position-error step to inject, radians */
    uint32_t duration_ms;/* how long to run the test before auto-stopping */
)

/* Device -> host: one sample of an in-progress test. Streamed at a paced
 * rate (decimated from the 500Hz control loop) so it doesn't saturate CDC. */
RELIABLE_PACKET(pidTelemPacket,
    uint32_t t_ms;    /* time since test start */
    float error;      /* position error fed into the PID (rad) */
    float output;     /* PID output (motor current command) */
    float integral;   /* integral term, useful for spotting windup */
)

/* Device -> host: sent once a test ends (duration elapsed or aborted).
 * Echoes the gains that were tested so the host can match it to the run
 * it requested even if commands overlap in flight. */
RELIABLE_PACKET(pidTestDonePacket,
    float kp;
    float ki;
    float kd;
    float kff;
    uint32_t sample_count;
)


/* ────────────────────────────────────────────────────────────────────────── */
/* Global Command Structures (accessible from other tasks) */
/* ────────────────────────────────────────────────────────────────────────── */

/* Aimbot/Gimbal command received over USB */
typedef struct {
    float pitch;
    float yaw;
    int32_t fire;       /* 0 = no fire, non-zero = fire (normalized to 1 in parser) */
    uint8_t tracking;   /* 1 = armor detected, 0 = not */
} aimbot_command_t;

extern aimbot_command_t g_aimbot_cmd;

/* Navigation/Chassis velocity command received over USB */
typedef struct {
    float vx;
    float vy;
    float vz;
    uint32_t last_update;
} nav_command_t;


extern nav_command_t g_nav_cmd;

/* USB connection status */
extern volatile uint8_t gv_usb_connected;

/* ────────────────────────────────────────────────────────────────────────── */
/* Statistics (optional, for debugging/monitoring) */
/* ────────────────────────────────────────────────────────────────────────── */
extern uint32_t g_usb_packet_count;  // total packets received
extern uint32_t g_usb_pps;           // packets per second (updated every 1s)

/* ────────────────────────────────────────────────────────────────────────── */
/* Public Functions */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the USB parser task and ring buffer
 * Call this from your main init (e.g. after MX_USB_DEVICE_Init())
 */
void USB_Firmware_Init(void);
void UsbParserTask(void *argument);

/**
 * @brief Internal: Called from usbd_cdc_if.c in CDC_Receive_FS
 * Do NOT call this manually.
 */
void usb_ring_buffer_write(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_INC_USB_TASK_H_ */
