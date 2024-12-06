/*
 * Telemetry.cpp
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#include "Telemetry.h"
#include "dummy_thread.h"

#include "typedefs.h"

#include "tim.h"
#include "spi.h"
#include "can.h"

#include "SuperCapCommThread.h"

ROCANDriver* CAN1_driver = nullptr;
CANBus* CAN1_network = nullptr;
ROCANDriver* CAN2_driver = nullptr;
CANBus* CAN2_network = nullptr;

dummyThread* dummy = nullptr;
SuperCapCommThread* supercap_thread = nullptr;
//imuThread* imu = nullptr;
//gimbalJointPubThread* gimbalJointThread = nullptr;
//ChassisSpdCmdThread* speedCmdThread = nullptr;
//statusThread* status = nullptr;

extern gimbal_control_t gimbal_ctrl_data;

void Telemetry::setup() {
		// UART line(s) initialization
//		UART1_line = new STMUARTDriver(&huart1);
//		UART1_network = new NetworkBus(UART1_line);

		// CANFD network initialization
		CAN1_driver = new ROCANDriver(&hcan1, CURRENT_NODE_ID);
		CAN1_network = new CANBus(CAN1_driver);

//		CAN2_driver = new ROCANDriver(&hcan2, CURRENT_NODE_ID);
//		CAN2_network = new CANBus(CAN2_driver);
		dummy = new dummyThread();
//		supercap_thread = new SuperCapCommThread();


		CAN1_network->handle<dummyPacket>(&dummyThread::handle_dummy);
		CAN1_network->handle<SuperCapDataPacket>(&SuperCapCommThread::handle_supercap);

//		gimbalJointThread = new gimbalJointPubThread();
////		imu = new imuThread();
//		speedCmdThread = new ChassisSpdCmdThread();
//		status = new statusThread();

//		UART1_network->handle<chassisSpeedCommandPacket>(&ChassisSpdCmdThread::handle_chassis_spd_commands);
//		UART1_network->handle<gimbalAngleCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_spd_commands);
//		UART1_network->handle<gimbalAnglePitchCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_pitch_command);
//		UART1_network->handle<gimbalAngleYawCommandPacket>(&ChassisSpdCmdThread::handle_gimbal_yaw_command);
//		UART1_network->handle<FrontFiringPacket>(&ChassisSpdCmdThread::handle_launcher_front_firing_commands);
//		UART1_network->handle<ChassisSpinCommandPacket>(&ChassisSpdCmdThread::handle_chassis_spin_command);


}

void Telemetry::set_id(uint32_t id) {
	dynamic_cast<ROCANDriver*>(CAN1_network->get_driver())->TxHeaderConfigID(id);
//	dynamic_cast<ROCANDriver*>(FDCAN2_network->get_driver())->TxHeaderConfigID(id);
}
