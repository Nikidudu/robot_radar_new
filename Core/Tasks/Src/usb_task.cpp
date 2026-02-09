/*
 * usb_task.cpp
 *
 *  Created on: Feb 9, 2026
 *      Author: AI Assistant
 */
#include "board_lib.h"
#include "usb_task.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include <string.h>

// Include BRoCo Protocol
#include "Protocol/Protocol.h"
#include "referee_msgs.h"

/* ────────────────────────────────────────────────────────────────────────── */
/* Protocol IDs for NUS25 (Matching NetworkBus.cpp) */
/* ────────────────────────────────────────────────────────────────────────── */
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
/* External Referee Data */
/* ────────────────────────────────────────────────────────────────────────── */
extern "C" {
    extern ref_game_state_t ref_game_state;
    extern ref_game_robot_data2_t ref_robot_data;
    extern ref_game_robot_HP_t ref_robot_hp;
}

/* ────────────────────────────────────────────────────────────────────────── */
/* Ring Buffer */
/* ────────────────────────────────────────────────────────────────────────── */
#define USB_RING_BUFFER_SIZE 2048  // Must be power of 2

aimbot_command_t g_aimbot_cmd = {0};
nav_command_t g_nav_cmd = {0};

static uint8_t usb_ring_buffer[USB_RING_BUFFER_SIZE];
static volatile uint32_t usb_rb_head = 0;
static volatile uint32_t usb_rb_tail = 0;

static inline uint32_t usb_rb_bytes_available(void)
{
    return (usb_rb_head - usb_rb_tail) & (USB_RING_BUFFER_SIZE - 1);
}

extern "C" void usb_ring_buffer_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t next_head = (usb_rb_head + 1) & (USB_RING_BUFFER_SIZE - 1);
        if (next_head == usb_rb_tail)
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
/* Helper Functions */
/* ────────────────────────────────────────────────────────────────────────── */

// Helper to send data with preamble and ID
template <typename T>
void USB_Send_Packet(uint8_t packet_id, T& packet)
{
    MAKE_RELIABLE(packet);
    
    uint8_t header[2];
    header[0] = USB_MAGIC_BYTE; // 0x7F
    header[1] = packet_id;
    
    // We send Header then Packet
    // CDC_Transmit_FS expects a contiguous buffer if we want one USB packet, 
    // but typically it handles multiple calls or we can copy to a buffer.
    // For safety and atomicity, let's copy to a temp buffer.
    
    uint8_t tx_buf[2 + sizeof(T)];
    tx_buf[0] = header[0];
    tx_buf[1] = header[1];
    memcpy(&tx_buf[2], &packet, sizeof(T));
    
    CDC_Transmit_FS(tx_buf, sizeof(tx_buf));
}

void USB_Send_GameStatus()
{
    competitionStatusPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.game_progress = ref_game_state.game_progress;
    packet.time_left = ref_game_state.stage_remain_time;
    packet.robot_id = ref_robot_data.robot_id;
    packet.current_hp = ref_robot_data.current_HP;
    
    // Robot HPs
    packet.red_hero_hp = ref_robot_hp.red_1_HP;
    packet.red_standard_hp = ref_robot_hp.red_3_HP; // Assuming Standard 1 is representative or we sum them? 
                                                   // Usually dashboard wants specific bots. 
                                                   // ProtocolNUS25 has singular 'standard_hp'? 
                                                   // Actually it has red_standard_hp. Maybe main standard?
                                                   // Let's use Standard 1 (ID 3) for now.
    packet.red_sentry_hp = ref_robot_hp.red_7_HP;
    
    packet.blue_hero_hp = ref_robot_hp.blu_1_HP;
    packet.blue_standard_hp = ref_robot_hp.blu_3_HP;
    packet.blue_sentry_hp = ref_robot_hp.blu_7_HP;

    USB_Send_Packet(ID_COMPETITION_STATUS, packet);
}

/* ────────────────────────────────────────────────────────────────────────── */
/* Packet Handler */
/* ────────────────────────────────────────────────────────────────────────── */
static void usb_handle_packet(uint8_t id, const uint8_t *payload, uint16_t len)
{
    switch (id)
    {
        case ID_CHASSIS_SPEED:
            if (len == sizeof(chassisSpeedCommandPacket))
            {
                chassisSpeedCommandPacket* pkt = (chassisSpeedCommandPacket*)payload;
                if (IS_RELIABLE(*pkt))
                {
                    g_nav_cmd.vx = pkt->V_horz;
                    g_nav_cmd.vy = pkt->V_lat;
                    g_nav_cmd.vz = pkt->V_yaw;
                    g_nav_cmd.last_update = HAL_GetTick();
                }
            }
            break;

        case ID_GIMBAL_COMMAND:
            if (len == sizeof(cvGimbalCommandPacket))
            {
                cvGimbalCommandPacket* pkt = (cvGimbalCommandPacket*)payload;
                if (IS_RELIABLE(*pkt))
                {
                    g_aimbot_cmd.yaw = pkt->yaw;
                    g_aimbot_cmd.pitch = pkt->pitch;
                }
            }
            break;

        case ID_FIRING_COMMAND:
            if (len == sizeof(firingCommandPacket))
            {
                firingCommandPacket* pkt = (firingCommandPacket*)payload;
                if (IS_RELIABLE(*pkt))
                {
                    g_aimbot_cmd.fire = pkt->fire_state ? 1 : 0;
                }
            }
            break;
            
        case ID_DUMMY:
            // Heartbeat or ping
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
    STATE_WAIT_PREAMBLE,
    STATE_WAIT_ID,
    STATE_WAIT_DATA
} usb_parse_state_t;

extern "C" void UsbParserTask(void *argument)
{
    usb_parse_state_t state = STATE_WAIT_PREAMBLE;
    uint8_t pkt_id = 0;
    uint16_t payload_len = 0;
    uint16_t payload_pos = 0;
    uint8_t payload_buf[USB_MAX_PAYLOAD_SIZE];

    uint32_t timeout_cnt = 0;
    uint32_t last_send_tick = 0;

    for (;;)
    {
        uint32_t tick = xTaskGetTickCount();
        
        // PPS Stats
        if (tick - last_stats_tick >= pdMS_TO_TICKS(1000))
        {
            g_usb_pps = g_usb_packet_count;
            g_usb_packet_count = 0;
            last_stats_tick = tick;
        }

        // Send Game Status @ 10Hz
        if (tick - last_send_tick >= pdMS_TO_TICKS(100))
        {
            USB_Send_GameStatus();
            last_send_tick = tick;
        }

        // Process Incoming Data
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
            case STATE_WAIT_PREAMBLE:
                if (byte == USB_MAGIC_BYTE) // 0x7F
                {
                    state = STATE_WAIT_ID;
                }
                break;

            case STATE_WAIT_ID:
                pkt_id = byte;
                // Determine length based on ID
                switch (pkt_id)
                {
                    case ID_CHASSIS_SPEED: payload_len = sizeof(chassisSpeedCommandPacket); break;
                    case ID_GIMBAL_COMMAND: payload_len = sizeof(cvGimbalCommandPacket); break;
                    case ID_FIRING_COMMAND: payload_len = sizeof(firingCommandPacket); break;
                    case ID_DUMMY: payload_len = sizeof(dummyPacket); break;
                    case ID_SURVEIL_COMMAND: payload_len = sizeof(surveilCommandPacket); break;
                    case ID_AIM_COMMAND: payload_len = sizeof(aimCommandPacket); break;
                    case ID_IS_NAVIGATING: payload_len = sizeof(isNavigatingPacket); break;
                    default: 
                        // Unknown ID, reset
                        state = STATE_WAIT_PREAMBLE; 
                        payload_len = 0;
                        break;
                }
                
                if (payload_len > 0)
                {
                    payload_pos = 0;
                    state = STATE_WAIT_DATA;
                }
                break;

            case STATE_WAIT_DATA:
                payload_buf[payload_pos++] = byte;
                if (payload_pos >= payload_len)
                {
                    usb_handle_packet(pkt_id, payload_buf, payload_len);
                    state = STATE_WAIT_PREAMBLE;
                }
                break;
        }

        // Timeout (~200ms)
        if (++timeout_cnt > 200)
        {
            gv_usb_connected = 0;
            timeout_cnt = 0;
            state = STATE_WAIT_PREAMBLE;
        }
    }
}

extern "C" void USB_Firmware_Init(void)
{
    xTaskCreate(UsbParserTask, "UsbParser", 512, NULL, 12, NULL);
}