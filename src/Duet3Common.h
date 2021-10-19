/*
 * Duet3Common.h
 *
 * This file defines the system-wide limits for Duet 3 that need to be known by main boards and expansion boards
 *
 *  Created on: 27 Dec 2019
 *      Author: David
 */

#ifndef SRC_DUET3COMMON_H_
#define SRC_DUET3COMMON_H_

#include <cstdint>
#include <cstddef>
#include <General/NamedEnum.h>

// Limits of Duet 3 systems
constexpr size_t MaxSensors = 56;						// limited by the size of bitmap we can store in an ExpressionValue
constexpr size_t MaxHeaters = 32;
constexpr size_t MaxMonitorsPerHeater = 3;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxZProbeProgramBytes = 8;				// maximum number of bytes in a Z probe program

constexpr size_t MaxFans = 20;

constexpr size_t MaxGpOutPorts = 32;					// increased as requested by Jimmykc

// The following currently don't need to be known by expansion boards, but might in future
constexpr size_t MaxGpInPorts = 16;
constexpr size_t MaxSpindles = 4;						// maximum number of configurable spindles

constexpr uint32_t ActLedFlashTime = 100;				// how long the ACT LED stays on after we process a CAN message

// The values of this enumeration must correspond to the meanings of the M569.1 S parameter
NamedEnum(EncoderType, uint8_t, none, linearQuadrature, rotaryQuadrature, AS5047, TLI5012);

// Error codes, presented as a number of flashes of the DIAG LED, used by both the bootloader and by expansion boards
enum class FirmwareFlashErrorCode : unsigned int
{
	ok = 0,
	invalidFirmware = 2,
	badCRC = 3,
	blockReceiveTimeout = 4,
	noFile = 5,
	badOffset = 6,
	hostOther = 7,
	noMemory = 8,
	flashInitFailed = 9,
	unlockFailed = 10,
	eraseFailed = 11,
	writeFailed = 12,
	lockFailed = 13,
	vinTooLow = 14,
	unknownBoard = 15
};

// Variables available for recording in closed-loop mode
constexpr uint16_t CL_RECORD_RAW_ENCODER_READING 			= 1u << 0;
constexpr uint16_t CL_RECORD_CURRENT_MOTOR_STEPS 			= 1u << 1;
constexpr uint16_t CL_RECORD_TARGET_MOTOR_STEPS 			= 1u << 2;
constexpr uint16_t CL_RECORD_CURRENT_ERROR 					= 1u << 3;
constexpr uint16_t CL_RECORD_PID_CONTROL_SIGNAL 			= 1u << 4;
constexpr uint16_t CL_RECORD_PID_P_TERM 					= 1u << 5;
constexpr uint16_t CL_RECORD_PID_I_TERM 					= 1u << 6;
constexpr uint16_t CL_RECORD_PID_D_TERM 					= 1u << 7;
constexpr uint16_t CL_RECORD_STEP_PHASE 					= 1u << 8;
constexpr uint16_t CL_RECORD_DESIRED_STEP_PHASE 			= 1u << 9;
constexpr uint16_t CL_RECORD_PHASE_SHIFT 					= 1u << 10;
constexpr uint16_t CL_RECORD_COIL_A_CURRENT 				= 1u << 11;
constexpr uint16_t CL_RECORD_COIL_B_CURRENT 				= 1u << 12;

#endif /* SRC_DUET3COMMON_H_ */
