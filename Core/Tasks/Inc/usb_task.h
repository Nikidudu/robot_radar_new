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

/* ────────────────────────────────────────────────────────────────────────── */
/* Protocol Constants */
/* ────────────────────────────────────────────────────────────────────────── */
#define USB_MAGIC_BYTE      0x7F
#define USB_MAX_PAYLOAD_SIZE 256


/* Packet Types */
// Deprecated constants kept for compatibility if needed, but new protocol uses IDs
#define USB_PKT_AIMBOT      0xA1
#define USB_PKT_NAV         0xA2
#define USB_PKT_HP_DATA		0xA3


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
