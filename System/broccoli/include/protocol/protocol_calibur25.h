/*
 * ProtocolNUS25.h
 *
 *  Created on: Jun 19, 2025
 *      Author: JL, CW
 */

#ifndef BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS25_H_
#define BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS25_H_

#include "protocol_macros.h"

#include <cstdint>

#include <vector>

// RELIABLE_PACKET(IMUPacket,
// float acceleration[3];				//[m/s^2]
// float angular[3];					//[°/s]
// float orientation[3];				//[rad]
// )
//
//RELIABLE_PACKET(MagPacket,
//float mag[3];
//float mag_raw[3];
//)
//
//RELIABLE_PACKET(gimbalJointsPacket,
//float yaw_angle;
//float pitch_angle;
//)
//
//RELIABLE_PACKET(chassisJointsPacket,
//float front_right_angle;
//float front_left_angle;
//float back_right_angle;
//float back_left_angle;
//)
//
//
RELIABLE_IDENTIFIABLE_PACKET(dummyPacket,
  int num1;
  int num2;
  int num3;
)
//
//RELIABLE_PACKET(imuPacket,
//  float pitch;
//  float yaw;
//)
//
RELIABLE_PACKET(chassisSpeedCommandPacket,
  float V_horz;
  float V_lat;
  float V_yaw;
)
RELIABLE_PACKET(chassisSpinCommandPacket,
  bool spinning_state;
)
//
//RELIABLE_PACKET(gimbalAngleCommandPacket,
//  float pitch;
//  float yaw;
//)
//
//RELIABLE_PACKET(gimbalAnglePitchCommandPacket,
//  float pitch;
//  float yaw;
//)
//
//RELIABLE_PACKET(gimbalAngleYawCommandPacket,
//  float yaw;
//)
//
RELIABLE_PACKET(leftTriggerPositionPacket,
 uint8_t trigger_pos; // [0: Undefined mode, 1: Sentry mode, 2: Predator mode, 3: Idle mode]
)
//
//RELIABLE_PACKET(RightTriggerPositionPacket,
//  uint8_t trigger_pos; // [0: Sentry control mode, 1: Manual control mode, 2: Sentry down]
//)
//

// Real-time data from microcontroller
RELIABLE_PACKET(competitionStatusPacket,
  uint16_t game_progress; // Which stage the competition is in
  uint16_t time_left; // Time left in competition (s)
  uint16_t robot_id;
  uint16_t current_hp;
  uint16_t red_hero_hp;
  uint16_t red_standard_hp;
  uint16_t red_sentry_hp;
  uint16_t blue_hero_hp;
  uint16_t blue_standard_hp;
  uint16_t blue_sentry_hp;
  uint8_t central_occupation;
  uint8_t resupply_occupation;
  bool win_state;
)

// Commands to microcontroller
RELIABLE_PACKET(cvGimbalCommandPacket,
  float yaw;
  float pitch;
)

RELIABLE_PACKET(firingCommandPacket,
  bool fire_state; // [0: Stop firing, 1: Start firing]
)

RELIABLE_PACKET(cvDetectedPacket,
  bool detected_state; // [0: Nothing detected, 1: Target detected]
)

RELIABLE_PACKET(aimCommandPacket,
  bool aiming_state; // [0: Stop aiming, 1: Start aiming]
)

RELIABLE_PACKET(isNavigatingPacket,
  bool navigating_state; // [0: Not navigating, 1: Is navigating]
)

#endif /* BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS25_H_ */
