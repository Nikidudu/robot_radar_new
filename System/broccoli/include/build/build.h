/*
 *  NUS Calibur Robotics
 *  build.h
 *
 *  Created on: 2 Nov 2024
 *      Author: JL
 *      Edited by : ______
 */

#ifndef BUILD_BUILD_H_
#define BUILD_BUILD_H_


#define PROTOCOL_25
#define BUILD_FOR_CALIBUR25


#if defined(BUILD_FOR_TESTING)
    #define IDK
#elif defined(BUILD_FOR_CALIBUR24) || defined(BUILD_FOR_CALIBUR25)
    #ifndef FREERTOS_ENABLED
	    #define FREERTOS_ENABLED
    #endif
	#define BUILD_WITH_STMUART
	#define BUILD_WITH_NETWORK_BUS
//	#define BUILD_WITH_UDEV_DRIVER
      #define BUILD_WITH_CAN
	//  #define BUILD_WITH_FDCAN
	//	#define BUILD_WITH_CAN_SOCKET_DRIVER
//		#define BUILD_WITH_CAN_BUS
	// #define BUILD_WITH_CACHES
#else
	#error "Please specify a build target"
#endif


#endif /* BUILD_BUILD_H_ */
