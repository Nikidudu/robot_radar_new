/*
 * usb_task.c
 *
 *  Created on: Feb 9, 2026
 *      Author: AI Assistant
 */
#include "board_lib.h"
#include "usb_task.h"
#include "pid_tuner_task.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────────────────── */
/* External Referee Data */
/* ────────────────────────────────────────────────────────────────────────── */
extern ref_game_state_t ref_game_state;
extern ref_game_robot_data_t ref_robot_data;
extern ref_game_robot_HP_ally_t ref_robot_hp;
extern orientation_data_t imu_heading;

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

void usb_ring_buffer_write(const uint8_t *data, uint32_t len)
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

/* ────────────────────3────────────────────────────────────────────────────── */
/* Helper Functions */
/* ────────────────────────────────────────────────────────────────────────── */

// Helper to send data with preamble and ID
void USB_Send_Raw(uint8_t packet_id, void* data, uint16_t size)
{
    uint8_t tx_buf[256];
    if (size > 250) return;

    tx_buf[0] = USB_MAGIC_BYTE; // 0x7F
    tx_buf[1] = packet_id;
    memcpy(&tx_buf[2], data, size);

    CDC_Transmit_FS(tx_buf, 2 + size);
}

/* State machine: 0=not started, 1-2=red (1=>80%, 2=<40%), 3-10=red spare,
 * 11-12=blue (11=>80%, 12=<40%), 13-20=blue spare.
 * Only updates when conditions change; otherwise keeps previous state. */
static uint8_t usb_compute_state(void)
{
    static uint8_t last_state = 0;

    uint8_t gp = ref_game_state.game_progress;
    uint16_t robot_id = ref_robot_data.robot_id;
    uint16_t cur_hp = ref_robot_data.current_HP;
    uint16_t max_hp = ref_robot_data.maximum_HP;

    uint8_t new_state;

    /* State 0: game not started (only stage 4 = in battle counts as started) */
    if (gp != 4) {
        new_state = 0;
    } else {
        /* Health percentage (avoid div by zero) */
        uint8_t hp_pct = (max_hp > 0) ? (uint8_t)((cur_hp * 100U) / max_hp) : 0;

        /* Red team: robot_id 1-9 */
        if (robot_id >= 1 && robot_id <= 9) {
            if (hp_pct > 80) new_state = 1;
            else if (hp_pct < 40) new_state = 2;
            else new_state = 3;  /* 40-80%: spare */
        }
        /* Blue team: robot_id 101-109 */
        else if (robot_id >= 101 && robot_id <= 109) {
            if (hp_pct > 80) new_state = 11;
            else if (hp_pct < 40) new_state = 12;
            else new_state = 13;  /* 40-80%: spare */
        }
        else {
            new_state = 0;  /* Unknown robot_id */
        }
    }

    /* Only update when condition changes */
    if (new_state != last_state) {
        last_state = new_state;
    }
    return last_state;
}

void USB_Send_GameStatus()
{
    competitionStatusPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.game_progress = ref_game_state.game_progress;
    packet.time_left = ref_game_state.stage_remain_time;
    packet.robot_id = ref_robot_data.robot_id;
    packet.current_hp = ref_robot_data.current_HP;

    /* Robot HPs */
    packet.red_hero_hp = ref_robot_hp.ally_1_robot_HP;
    packet.red_standard_hp = ref_robot_hp.ally_3_robot_HP;
    packet.red_sentry_hp = ref_robot_hp.ally_7_robot_HP;

    packet.blue_hero_hp = ref_robot_hp.ally_1_robot_HP;
    packet.blue_standard_hp = ref_robot_hp.ally_3_robot_HP;
    packet.blue_sentry_hp = ref_robot_hp.ally_7_robot_HP;

    MAKE_RELIABLE(packet);
    USB_Send_Raw(ID_COMPETITION_STATUS, &packet, sizeof(packet));
}

void USB_Send_State(void)
{
    statePacket packet = {0};
    packet.state = usb_compute_state();
    MAKE_RELIABLE(packet);
    USB_Send_Raw(ID_STATE, &packet, sizeof(packet));
}

void USB_Send_GimbalStatus()
{
    gimbalJointsPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.yaw_angle = imu_heading.yaw;
    packet.pitch_angle = imu_heading.pit;

    MAKE_RELIABLE(packet);
    USB_Send_Raw(ID_GIMBAL_JOINTS, &packet, sizeof(packet));
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
                    g_nav_cmd.vy = -(pkt->V_lat);
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
                    g_aimbot_cmd.pitch = -(pkt->pitch);
                    g_aimbot_cmd.yaw = -(pkt->yaw);
                    g_aimbot_cmd.tracking = pkt->tracking;
                }
            }
            break;

        case ID_FIRING_COMMAND:
            if (len == sizeof(firingCommandPacket))
            {
                firingCommandPacket* pkt = (firingCommandPacket*)payload;
                if (IS_RELIABLE(*pkt))
                {
                    g_aimbot_cmd.fire = (pkt->fire_state != 0) ? 1 : 0;
                }
            }
            break;

        case ID_PID_TEST_CMD:
            if (len == sizeof(pidTestCmdPacket))
            {
                pidTestCmdPacket* pkt = (pidTestCmdPacket*)payload;
                if (IS_RELIABLE(*pkt))
                {
                    pid_tuner_handle_cmd(pkt);
                }
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
    STATE_WAIT_PREAMBLE,
    STATE_WAIT_ID,
    STATE_WAIT_DATA
} usb_parse_state_t;

void UsbParserTask(void *argument)
{
    usb_parse_state_t state = STATE_WAIT_PREAMBLE;
    uint8_t pkt_id = 0;
    uint16_t payload_len = 0;
    uint16_t payload_pos = 0;
    uint8_t payload_buf[USB_MAX_PAYLOAD_SIZE];

    uint32_t timeout_cnt = 0;
    uint32_t last_game_send_tick = 0;
    uint32_t last_state_send_tick = pdMS_TO_TICKS(50);  /* Stagger 50ms to avoid USBD_BUSY */
    uint32_t last_gimbal_send_tick = 0;

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
        if (tick - last_game_send_tick >= pdMS_TO_TICKS(100))
        {
            USB_Send_GameStatus();
            last_game_send_tick = tick;
        }

        // Send State @ 50Hz, staggered from GameStatus (CDC single-buffered)
        if (tick - last_state_send_tick >= pdMS_TO_TICKS(20))
        {
            USB_Send_State();
            last_state_send_tick = tick;
        }

        // Send Gimbal Status @ 50Hz for smooth tracking
        if (tick - last_gimbal_send_tick >= pdMS_TO_TICKS(20))
        {
            USB_Send_GimbalStatus();
            last_gimbal_send_tick = tick;
        }

        // Drain any buffered PID-tuner telemetry (see pid_tuner_task.c).
        // Capped per iteration so a full ring doesn't monopolize the loop;
        // production is ~250Hz and this loop runs at least every ~1ms, so
        // a small cap here still drains faster than samples are produced.
        {
            pidTelemPacket sample;
            for (uint8_t i = 0; i < 8 && pid_tuner_pop_sample(&sample); i++)
            {
                MAKE_RELIABLE(sample);
                USB_Send_Raw(ID_PID_TELEM, &sample, sizeof(sample));
            }

            pidTestDonePacket done;
            if (pid_tuner_pop_done(&done))
            {
                MAKE_RELIABLE(done);
                USB_Send_Raw(ID_PID_TEST_DONE, &done, sizeof(done));
            }
        }

        // Process all available data in one go for lowest latency
        uint32_t avail = usb_rb_bytes_available();
        if (avail == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            timeout_cnt++;
            if (timeout_cnt > 200)
            {
                gv_usb_connected = 0;
                timeout_cnt = 0;
                state = STATE_WAIT_PREAMBLE;
            }
            continue;
        }

        timeout_cnt = 0;
        gv_usb_connected = 1;

        while (avail--)
        {
            uint8_t byte = usb_ring_buffer[usb_rb_tail];
            usb_rb_tail = (usb_rb_tail + 1) & (USB_RING_BUFFER_SIZE - 1);

            switch (state)
            {
                case STATE_WAIT_PREAMBLE:
                    if (byte == USB_MAGIC_BYTE)
                    {
                        state = STATE_WAIT_ID;
                    }
                    break;

                case STATE_WAIT_ID:
                    pkt_id = byte;
                    switch (pkt_id)
                    {
                        case ID_CHASSIS_SPEED: payload_len = sizeof(chassisSpeedCommandPacket); break;
                        case ID_GIMBAL_COMMAND: payload_len = sizeof(cvGimbalCommandPacket); break;
                        case ID_FIRING_COMMAND: payload_len = sizeof(firingCommandPacket); break;
                        case ID_DUMMY: payload_len = sizeof(dummyPacket); break;
                        case ID_SURVEIL_COMMAND: payload_len = sizeof(surveilCommandPacket); break;
                        case ID_AIM_COMMAND: payload_len = sizeof(aimCommandPacket); break;
                        case ID_IS_NAVIGATING: payload_len = sizeof(isNavigatingPacket); break;
                        case ID_PID_TEST_CMD: payload_len = sizeof(pidTestCmdPacket); break;
                        default:
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
        }
    }
}

void USB_Firmware_Init(void)
{
    pid_tuner_init();
    xTaskCreate(UsbParserTask, "UsbParser", 512, NULL, 12, NULL);
}
