/*
 * usb_config_task.c
 *
 *  Created on: Dec 20, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "usb_task.h"
#include "task.h"
#include <string.h>

/* ────────────────────────────────────────────────────────────────────────── */
/* Ring Buffer */
/* ────────────────────────────────────────────────────────────────────────── */
#define USB_RING_BUFFER_SIZE 2048  // Must be power of 2

aimbot_command_t g_aimbot_cmd = {0};
nav_command_t g_nav_cmd = {0};

static uint8_t usb_ring_buffer[USB_RING_BUFFER_SIZE];
static volatile uint32_t usb_rb_head = 0;  // Written by ISR/callback
static volatile uint32_t usb_rb_tail = 0;  // Read by parser task

static inline uint32_t usb_rb_bytes_available(void)
{
    return (usb_rb_head - usb_rb_tail) & (USB_RING_BUFFER_SIZE - 1);
}

void usb_ring_buffer_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t next_head = (usb_rb_head + 1) & (USB_RING_BUFFER_SIZE - 1);
        if (next_head == usb_rb_tail)  // Buffer full → overwrite oldest
        {
            usb_rb_tail = (usb_rb_tail + 1) & (USB_RING_BUFFER_SIZE - 1);
        }
        usb_ring_buffer[usb_rb_head] = data[i];
        usb_rb_head = next_head;
    }
}

/* ────────────────────────────────────────────────────────────────────────── */
/* Globals */
/* ────────────────────────────────────────────────────────────────────────── */
volatile uint8_t gv_usb_connected = 0;

uint32_t g_usb_packet_count = 0;
uint32_t g_usb_pps = 0;
static uint32_t last_stats_tick = 0;

/* ────────────────────────────────────────────────────────────────────────── */
/* CRC16-CCITT (0xFFFF init, 0x1021 poly) */
/* ────────────────────────────────────────────────────────────────────────── */
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ────────────────────────────────────────────────────────────────────────── */
/* Packet Handler */
/* ────────────────────────────────────────────────────────────────────────── */
static void usb_handle_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    switch (type)
    {
        case USB_PKT_AIMBOT:
            if (len == 12)  // 3 x float
            {
                memcpy(&g_aimbot_cmd.yaw,   payload + 0,  4);
                memcpy(&g_aimbot_cmd.pitch, payload + 4,  4);
                memcpy(&g_aimbot_cmd.fire,  payload + 8,  4);
                if (g_aimbot_cmd.fire != 0) g_aimbot_cmd.fire = 1;
            }
            break;

        case USB_PKT_NAV:
            if (len == 12)  // vx, vy, vz
            {
                memcpy(&g_nav_cmd.vx, payload + 0, 4);
                memcpy(&g_nav_cmd.vy, payload + 4, 4);
                memcpy(&g_nav_cmd.vz, payload + 8, 4);
            }
            break;

        default:
            break;
    }
    g_usb_packet_count++;
}

/* ────────────────────────────────────────────────────────────────────────── */
/* Parser Task */
/* ────────────────────────────────────────────────────────────────────────── */
typedef enum {
    STATE_WAIT_MAGIC,
    STATE_LEN_LO,
    STATE_LEN_HI,
    STATE_TYPE,
    STATE_PAYLOAD,
    STATE_CRC_LO,
    STATE_CRC_HI
} usb_parse_state_t;

void UsbParserTask(void *argument)
{
    usb_parse_state_t state = STATE_WAIT_MAGIC;
    uint16_t payload_len = 0;
    uint8_t pkt_type = 0;
    uint16_t crc_recv = 0;
    uint16_t payload_pos = 0;
    uint16_t header_pos = 0;

    uint8_t header_buf[4 + USB_MAX_PAYLOAD_SIZE];  // magic + len(2) + type + payload
    uint8_t payload_buf[USB_MAX_PAYLOAD_SIZE];

    uint32_t timeout_cnt = 0;

    for (;;)
    {
        // Update PPS every second
        uint32_t tick = xTaskGetTickCount();
        if (tick - last_stats_tick >= pdMS_TO_TICKS(1000))
        {
            g_usb_pps = g_usb_packet_count;
            g_usb_packet_count = 0;
            last_stats_tick = tick;
        }

        // Wait for data
        if (usb_rb_bytes_available() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint8_t byte = usb_ring_buffer[usb_rb_tail];
        usb_rb_tail = (usb_rb_tail + 1) & (USB_RING_BUFFER_SIZE - 1);

        timeout_cnt = 0;
        gv_usb_connected = 1;

        switch (state)
        {
            case STATE_WAIT_MAGIC:
                if (byte == USB_MAGIC_BYTE)
                {
                    state = STATE_LEN_LO;
                    header_pos = 0;
                    payload_pos = 0;
                    header_buf[header_pos++] = byte;
                }
                break;

            case STATE_LEN_LO:
                payload_len = byte;
                header_buf[header_pos++] = byte;
                state = STATE_LEN_HI;
                break;

            case STATE_LEN_HI:
                payload_len |= (uint16_t)byte << 8;
                if (payload_len > USB_MAX_PAYLOAD_SIZE)
                {
                    state = STATE_WAIT_MAGIC;
                    break;
                }
                header_buf[header_pos++] = byte;
                state = STATE_TYPE;
                break;

            case STATE_TYPE:
                pkt_type = byte;
                header_buf[header_pos++] = byte;
                state = (payload_len == 0) ? STATE_CRC_LO : STATE_PAYLOAD;
                break;

            case STATE_PAYLOAD:
                payload_buf[payload_pos++] = byte;
                header_buf[header_pos++] = byte;
                if (payload_pos >= payload_len)
                    state = STATE_CRC_LO;
                break;

            case STATE_CRC_LO:
                crc_recv = byte;
                state = STATE_CRC_HI;
                break;

            case STATE_CRC_HI:
                crc_recv |= (uint16_t)byte << 8;
                if (crc16(header_buf, header_pos) == crc_recv)
                {
                    usb_handle_packet(pkt_type, payload_buf, payload_len);
                }
                state = STATE_WAIT_MAGIC;
                break;

            default:
                state = STATE_WAIT_MAGIC;
                break;
        }

        // Timeout detection (~200ms no data)
        if (++timeout_cnt > 200)
        {
            gv_usb_connected = 0;
            timeout_cnt = 0;
            state = STATE_WAIT_MAGIC;
        }
    }
}


/* ────────────────────────────────────────────────────────────────────────── */
/* Initialization */
/* ────────────────────────────────────────────────────────────────────────── */
void USB_Firmware_Init(void)
{
    xTaskCreate(UsbParserTask, "UsbParser", 512, NULL, 12, NULL);
}
