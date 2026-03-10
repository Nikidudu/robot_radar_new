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

RELIABLE_PACKET(cvGimbalCommandPacket,
    float yaw;
    float pitch;
)

RELIABLE_PACKET(firingCommandPacket,
    bool fire_state;
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


/* ────────────────────────────────────────────────────────────────────────── */
/* Global Command Structures (accessible from other tasks) */
/* ────────────────────────────────────────────────────────────────────────── */

/* Aimbot/Gimbal command received over USB */
typedef struct {
    float yaw;      // radians or degrees — as sent from PC
    float pitch;
    int32_t fire;   // 0 = no fire, non-zero = fire (normalized to 1 in parser)
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
