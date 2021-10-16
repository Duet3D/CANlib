/*
 * Duet3Limits.h
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
constexpr size_t MaxSpindles = 2;						// maximum number of configurable spindles

// Other constants etc. that are common across Duet 3 main and expansion boards
constexpr float DefaultThermistorR25 = 100000.0;
constexpr float DefaultThermistorBeta = 4725.0;
constexpr float DefaultThermistorC = 7.060e-8;

constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

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

// Firmware module numbers
enum class FirmwareModule : uint8_t
{
	main = 0,
	wifi = 1,
	reserved = 2,
	bootloader = 3
};

NamedEnum(FilamentSensorStatus, uint8_t,
	noMonitor,
	ok,
	noDataReceived,
	noFilament,
	tooLittleMovement,
	tooMuchMovement,
	sensorError
);

NamedEnum(LogLevel, uint8_t, off, warn, info, debug);

// Meaning of the driver status bits. The lowest 8 bits of these have the same bit positions as in the TMC2209 DRV_STATUS register. The TMC5160 DRV_STATUS is different.
union StandardDriverStatus
{
	uint32_t all;
	struct
	{
		uint32_t otpw : 1,						// over temperature warning
				 ot : 1,						// over temperature error
				 s2ga : 1,						// short to ground phase A
				 s2gb : 1,						// short to ground phase B
				 s2vsa : 1,						// short to VS phase A
				 s2vsb : 1,						// short to VS phase B
				 ola : 1,						// open load phase A
				 olb : 1,						// open load phase B
				 // The remaining bit assignments do not correspond to TMC2209 bit positions
				 standstill : 1,				// standstill indicator
				 prestall : 1,					// close to stall, or closed loop warning
				 stall : 1,						// stall, or closed loop error exceeded
				 closedLoopNotTuned : 1,		// move attempted when closed loop driver not tuned
				 closedLoopTuningError : 1,		// closed loop tuning status
				 zero2 : 3,						// spare, always zero for now
				 sgresult : 10,					// reserved for stallguard result
				 zero3 : 6;						// reserved for future use
	};

	static constexpr unsigned int StandstillBitPos = 8;
	static constexpr unsigned int StallBitPos = 10;
	static constexpr unsigned int SgresultBitPos = 16;
};

static_assert(sizeof(StandardDriverStatus) == sizeof(uint32_t));

// Structure to represent the minimum, current and maximum values of a floating point quantity
struct MinCurMax
{
	float minimum;
	float current;
	float maximum;
};

// Enum to represent a heater state
enum class HeaterMode : uint8_t
{
	// The order of these is important because we test "mode > HeatingMode::suspended" to determine whether the heater is active
	// and "mode >= HeatingMode::off" to determine whether the heater is either active or suspended
	fault,
	offline,
	off,
	suspended,
	heating,
	cooling,
	stable,
	// All states from here onwards must be PID tuning states because function IsTuning assumes that
	tuning0,
	tuning1,
	tuning2,
	tuning3,
	firstTuningMode = tuning0,
	lastTuningMode = tuning3
};

// Variables available for recording in closed-loop mode
constexpr uint16_t CL_RECORD_RAW_ENCODER_READING 			= 1 << 0;
constexpr uint16_t CL_RECORD_CURRENT_MOTOR_STEPS 			= 1 << 1;
constexpr uint16_t CL_RECORD_TARGET_MOTOR_STEPS 			= 1 << 2;
constexpr uint16_t CL_RECORD_CURRENT_ERROR 					= 1 << 3;
constexpr uint16_t CL_RECORD_PID_CONTROL_SIGNAL 			= 1 << 4;
constexpr uint16_t CL_RECORD_PID_P_TERM 					= 1 << 5;
constexpr uint16_t CL_RECORD_PID_I_TERM 					= 1 << 6;
constexpr uint16_t CL_RECORD_PID_D_TERM 					= 1 << 7;
constexpr uint16_t CL_RECORD_STEP_PHASE 					= 1 << 8;
constexpr uint16_t CL_RECORD_DESIRED_STEP_PHASE 			= 1 << 9;
constexpr uint16_t CL_RECORD_PHASE_SHIFT 					= 1 << 10;
constexpr uint16_t CL_RECORD_COIL_A_CURRENT 				= 1 << 11;
constexpr uint16_t CL_RECORD_COIL_B_CURRENT 				= 1 << 12;

#endif /* SRC_DUET3COMMON_H_ */
