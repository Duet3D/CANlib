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
#include <General/Bitmap.h>

// Limits of Duet 3 systems
constexpr size_t MaxSensors = 56;							// limited by the size of bitmap we can store in an ExpressionValue
constexpr size_t MaxHeaters = 32;
constexpr size_t MaxMonitorsPerHeater = 3;
constexpr size_t MaxZProbes = 8;
constexpr size_t MaxFans = 32;
constexpr size_t MaxGpOutPorts = 64;						// increased in RRF 3.5.0-beta.4
constexpr size_t MaxLedStrips = 5;

constexpr unsigned int MaxLinearDriversPerCanSlave = 8;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

typedef Bitmap<uint16_t> LocalDriversBitmap;		// Type of a bitmap representing a set of driver numbers
typedef Bitmap<uint32_t> HeatersBitmap;				// Type of a bitmap representing a set of heater numbers
typedef Bitmap<uint32_t> FansBitmap;				// Type of a bitmap representing a set of fan numbers
typedef Bitmap<uint64_t> SensorsBitmap;				// Type of a bitmap representing sensors

static_assert(LocalDriversBitmap::MaxBits() >= MaxLinearDriversPerCanSlave);
static_assert(MaxHeaters <= HeatersBitmap::MaxBits());
static_assert(MaxFans <= FansBitmap::MaxBits());
static_assert(MaxSensors <= SensorsBitmap::MaxBits());

// The following currently don't need to be known by expansion boards, but might in future
constexpr size_t MaxGpInPorts = 56;							// increased in RRF 3.5.0-beta.4, limit this to 56 so that we can report trigger input bitmaps in the object model
constexpr size_t MaxSpindles = 4;							// maximum number of configurable spindles

constexpr uint32_t ActLedFlashTime = 100;					// how long the ACT LED stays on after we process a CAN message

constexpr uint32_t BasicDriverPositionRevertMillis = 40;	// how long we tell CAN-connected drivers that they have to revert their position after a move involving endstops
constexpr uint32_t TotalDriverPositionRevertMillis = BasicDriverPositionRevertMillis + 10;		// the same plus an allowance for how long it takes to send the CAN messages

// The values of this enumeration must correspond to the meanings of the M569.1 S parameter
NamedEnum(EncoderType, uint8_t, none, linearComposite, rotaryQuadrature, rotaryMagnetic);

// Error codes, presented as a number of flashes of the DIAG LED, used by both the bootloader and by expansion boards
enum class FirmwareFlashErrorCode : unsigned int
{
	ok = 0,								// used as a function return code, not used to flash the LED
	invalidFirmware = 2,				// bootloader determined that the installed firmware is invalid
	badCRC = 3,							// bootloader determined that the installed firmware fails the CRC check
	blockReceiveTimeout = 4,			// bootloader requested a firmware block from the master but didn't receive a reply
	noFile = 5,							// bootloader requested firmware but the master responded that it didn't have the requested file
	badOffset = 6,						// master reported that the file offset requested by the bootloader is out of range
	hostOther = 7,						// master reported it was unable to supply the requested block for some other reason
	noMemory = 8,						// bootloader ran out of RAM
	flashInitFailed = 9,				// failed to initialise flash memory
	unlockFailed = 10,					// failed to unlock flash memory
	eraseFailed = 11,					// failed to erase flash memory
	writeFailed = 12,					// failed to write flash memory
	lockFailed = 13,					// failed to lock flash memory
	vinTooLow = 14,						// expansion board was asked to update the bootloader but VIN is too low to do that safely
	unknownBoard = 15,					// bootloader failed to identify the board type
	vAssertCalled = 16,					// assertion failure in the bootloader
	noTimeSyncMessageSeen = 17			// bootloader didn't hear a clock message at any of the standard speeds (added for new bootloader)
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
constexpr uint16_t CL_RECORD_CURRENT_STEP_PHASE 			= 1u << 8;
constexpr uint16_t CL_RECORD_DESIRED_STEP_PHASE 			= 1u << 9;
constexpr uint16_t CL_RECORD_PHASE_SHIFT 					= 1u << 10;
constexpr uint16_t CL_RECORD_COIL_A_CURRENT 				= 1u << 11;
constexpr uint16_t CL_RECORD_COIL_B_CURRENT 				= 1u << 12;
constexpr uint16_t CL_RECORD_PID_V_TERM 					= 1u << 13;
constexpr uint16_t CL_RECORD_PID_A_TERM 					= 1u << 14;

#ifndef FLOAT16_T_DEFINED
# define FLOAT16_T_DEFINED
typedef __fp16 float16_t;			///< A 16-bit floating point type
#endif

// Total of the above is currently 34 bytes, plus 4 bytes for the time stamp = 38 bytes.
// We can fit 56 bytes of data in each CAN data sample message.

// Calculate how much data there is from the bitmap of data to collect
constexpr uint8_t ClosedLoopSampleLength(uint16_t valuesToCollect) noexcept
{
	// Size of each data item, in the same order as the CL_RECORD_ values declared in Duet3Common.h
	constexpr uint8_t ClosedLoopDataSizes[16] =
	{
		sizeof(int32_t),	// raw encoder reading
		sizeof(float),		// current motor steps
		sizeof(float),		// target motor steps
		sizeof(float),		// current error
		sizeof(float16_t),	// total PID control signal
		sizeof(float16_t),	// PID P term
		sizeof(float16_t),	// PID I term
		sizeof(float16_t),	// PID D term
		sizeof(uint16_t),	// current step phase
		sizeof(uint16_t),	// desired step phase
		sizeof(float16_t),	// phase shift
		sizeof(int16_t),	// coil A current
		sizeof(int16_t),	// coil B current
		sizeof(float16_t),	// PID V term
		sizeof(float16_t),	// PID A term
		0					// unused
	};

	uint8_t ret = sizeof(float);									// space for the time stamp
	for (unsigned int i = 0; valuesToCollect != 0 ; ++i)
	{
		if ((valuesToCollect & 1u) != 0)
		{
			ret += ClosedLoopDataSizes[i];
		}
		valuesToCollect >>= 1;
	}
	return ret;
}

constexpr size_t MaxClosedLoopSampleLength = ClosedLoopSampleLength(0xFFFF);

#endif /* SRC_DUET3COMMON_H_ */
