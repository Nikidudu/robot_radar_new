/*
 * Telemetry.cpp
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#include "Telemetry.h"

#include "typedefs.h"

#include "tim.h"
#include "spi.h"
#include "can.h"

#include "SuperCapCommThread.h"
#include "cvGimbalThread.h"
#include "cvAimbotCommandThread.h"
#include "status_thread.h"
#include "spd_cmd_thread.h"
#include "dummy_cmd_thread.h"



ROCANDriver* CAN1_driver = nullptr;
CANBus* CAN1_network = nullptr;
ROCANDriver* CAN2_driver = nullptr;
CANBus* CAN2_network = nullptr;

STMUARTDriver* UART_line = nullptr;
NetworkBus* UART_network = nullptr;

SuperCapCommThread* supercap_thread = nullptr;
cvGimbalThread* cv_gimbal_thread = nullptr;
cvAimbotCommandThread* cv_aimbot_command_thread = nullptr;
statusThread* status_thread = nullptr;

//imuThread* imu = nullptr;
//gimbalJointPubThread* gimbalJointThread = nullptr;
ChassisSpdCmdThread* speedCmdThread = nullptr;
dummyCmdThread* dummy_cmd = nullptr;
//statusThread* status = nullptr;

extern gimbal_control_t gimbal_ctrl_data;

void Telemetry::setup() {
//		 UART line(s) initialization
		UART_line = new STMUARTDriver(&huart1);
		UART_network = new NetworkBus(UART_line);

		// CANFD network initialization
		CAN1_driver = new ROCANDriver(&hcan1, DEVC_NODE_ID);
//		CAN1_network = new CANBus(CAN1_driver);

//		CAN2_driver = new ROCANDriver(&hcan2, CURRENT_NODE_ID);
//		CAN2_network = new CANBus(CAN2_driver);
//		dummy = new dummyThread();
//		supercap_thread = new SuperCapCommThread();
//		supercap_thread = new SuperCapCommThread();
//		CAN1_network->handle<SuperCapDataPacket>(&SuperCapCommThread::handle_supercap);

		// To send real-time data to mini PC
		status_thread = new statusThread();

		/* Aimbot threads
		 * One thread handles the callback for 3 packets
		 * cvGimbalCommandPacket - Pitch and yaw control
		 * cvFiringCommandPacket - Firing control
		 * cvAimCommandPacket -
		 */
		cv_aimbot_command_thread = new cvAimbotCommandThread();
		UART_network->handle<cvGimbalCommandPacket>(&cvAimbotCommandThread::handle_cv_gimbal);
		UART_network->handle<cvFiringCommandPacket>(&cvAimbotCommandThread::handle_cv_firing);
		UART_network->handle<cvAimCommandPacket>(&cvAimbotCommandThread::handle_cv_aim);

		// Sentry nav threads
//		gimbalJointThread = new gimbalJointPubThread();
//		imu = new imuThread();
		speedCmdThread = new ChassisSpdCmdThread();
		dummy_cmd = new dummyCmdThread();

		UART_network->handle<chassisSpeedCommandPacket>(&ChassisSpdCmdThread::handle_chassis_spd_commands);
//		UART1_network->handle<gimbalAngleCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_spd_commands);
//		UART1_network->handle<gimbalAnglePitchCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_pitch_command);
//		UART1_network->handle<gimbalAngleYawCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_yaw_command);
//		UART1_network->handle<FrontFiringPacket>(&ChassisSpdCmdThread::handle_launcher_front_firing_commands);
//		UART1_network->handle<ChassisSpinCommandPacket>(&ChassisSpdCmdThread::handle_chassis_spin_command);
		UART_network->handle<dummyPacket>(&dummyCmdThread::handle_dummy_cmd);


}

void Telemetry::set_id(uint32_t id) {
	dynamic_cast<ROCANDriver*>(CAN1_network->get_driver())->TxHeaderConfigID(id);
//	dynamic_cast<ROCANDriver*>(FDCAN2_network->get_driver())->TxHeaderConfigID(id);
}
