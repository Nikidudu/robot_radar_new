/*
 *  NUS Calibur Robotics
 *  protocol.h
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 *      Edited by : ______
 */

/*
 * ProtocolNUS24.h
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#ifndef BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_
#define BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_

#include "protocol_macros.h"

#include <cstdint>

#include <vector>

// RELIABLE_PACKET(IMUPacket,
// float acceleration[3];				//[m/s^2]
// float angular[3];					//[°/s]
// float orientation[3];				//[rad]
// )

// RELIABLE_PACKET(MagPacket,
// float mag[3];
// float mag_raw[3];
// )

// RELIABLE_PACKET(gimbalJointsPacket,
// float yaw_angle;
// float pitch_angle;
// )

// RELIABLE_PACKET(chassisJointsPacket,
// float front_right_angle;
// float front_left_angle;
// float back_right_angle;
// float back_left_angle;
// )

RELIABLE_PACKET(AccelConfigRequestPacket,
  bool req_bias;
  bool req_transform;
)

RELIABLE_PACKET(AccelConfigPacket,
  float bias[3];
  float transform[9];
  bool remote_command;
  bool set_bias;
  bool set_transform;
)

// Total size: 2 (preamble + packet ID) + 2 (id) + 12*4 (bias + transform) + 3 (bools) + 2 (crc) = 57 bytes
RELIABLE_PACKET(AccelConfigResponsePacket,
  float bias[3];
  float transform[9];
  bool set_bias;
  bool set_transform;
  bool success;
)

RELIABLE_PACKET(GyroConfigRequestPacket,
  bool req_bias;
)

RELIABLE_PACKET(GyroConfigPacket,
  float bias[3];
  bool remote_command;
  bool set_bias;
)

RELIABLE_PACKET(GyroConfigResponsePacket,
  float bias[3];
  bool set_bias;
  bool success;
)

RELIABLE_PACKET(MagConfigRequestPacket,
  bool req_hard_iron;
  bool req_soft_iron;
)

RELIABLE_PACKET(MagConfigPacket,
  float hard_iron[3];
  float soft_iron[9];
  bool remote_command;
  bool set_hard_iron;
  bool set_soft_iron;
)

// Total size: 2 (preamble + packet ID) + 2 (id) + 12*4 (hard_iron + soft_iron) + 3 (bools) + 2 (crc) = 57 bytes
RELIABLE_PACKET(MagConfigResponsePacket,
  float hard_iron[3];
  float soft_iron[9];
  bool set_hard_iron;
  bool set_soft_iron;
  bool success;
)

RELIABLE_PACKET(dummyPacket,
  int num1;
  int num2;
  int num3;
)

// RELIABLE_PACKET(imuPacket,
//   float pitch;
//   float yaw;
// )

// RELIABLE_PACKET(chassisSpeedCommandPacket,
//   float V_horz;
//   float V_lat;
//   float V_yaw;
// )

// RELIABLE_PACKET(gimbalAngleCommandPacket,
//   float pitch;
//   float yaw;
// )

// RELIABLE_PACKET(gimbalAnglePitchCommandPacket,
//   float pitch;
//   float yaw;
// )

// RELIABLE_PACKET(gimbalAngleYawCommandPacket,
//   float yaw;
// )

// RELIABLE_PACKET(LeftTriggerPositionPacket,
//   uint8_t trigger_pos; // [0: Attack mode, 1: Balance mode, 2: Sandbox mode]
// )

// RELIABLE_PACKET(RightTriggerPositionPacket,
//   uint8_t trigger_pos; // [0: Sentry control mode, 1: Manual control mode, 2: Sentry down]
// )

// RELIABLE_PACKET(FrontFiringPacket,
// bool fire_state; // Tells the front launcher to shoot or not (1 : True, 0 : False)
// )

// RELIABLE_PACKET(BackFiringPacket,
// bool fire_state; // Tells the back launcher to shoot or not (1 : True, 0 : False)
// )

// RELIABLE_PACKET(ChassisSpinCommandPacket,
// bool spining_state; // Tells the chassis to spin (1 : True, 0 : False)
// )

// RELIABLE_PACKET(RobotStatusPacket,
// //  uint16_t competition_time; // Time of the match
//   uint16_t game_progress; // Which stage the competition is in
//   uint16_t time_left; // Time left in competition (s)
// //  uint16_t red_base_hp; // HP of red base
// //  uint16_t blue_base_hp; // HP of blue base
// //  uint16_t virtual_shield_hp; // HP of virtual shield (% in integer)
//   uint16_t robot_id;
//   uint16_t current_hp;
//   uint16_t occupy_central;
//   uint16_t ammo;
// )

<<<<<<< Updated upstream:System/broccoli/include/protocol/protocol_calibur24.h
=======
// RELIABLE_PACKET(SideDialPacket,
//   bool robot_mode;  // [0: Attack mode, 1: Balance mode]
// )

// RELIABLE_IDENTIFIABLE_PACKET(SuperCapDataPacket,
//   float V_cap;
//   float P_chassis;
//   uint8_t charge_state;
// )

// RELIABLE_IDENTIFIABLE_PACKET(MaxChassisPowerPacket,
//   uint8_t max_chassis_power;
// )
>>>>>>> Stashed changes:System/BRoCo/include/Protocol/ProtocolNUS24.h



#endif /* BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_ */
