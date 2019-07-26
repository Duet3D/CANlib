/*
 * CanMessageFormats.h
 *
 *  Created on: 16 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEFORMATS_H_
#define SRC_CAN_CANMESSAGEFORMATS_H_

#include "CanId.h"
#include "General/BitMap.h"
#include "General/StringRef.h"

constexpr unsigned int MaxDriversPerCanSlave = 4;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

// CAN message formats

// Time sync message
struct CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	uint32_t timeSent;									// when this message was sent
	uint32_t lastTimeSent;								// when we tried to send the previous message
	uint32_t lastTimeAcknowledged;						// when the previous message was acknowledged
};

// Emergency stop message
struct CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;
};

// Movement message
struct CanMessageMovement
{
	static constexpr CanMessageType messageType = CanMessageType::movement;

	uint32_t whenToExecute;
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;

	uint32_t deltaDrives : 4,				// which drivers are doing delta movement
			 pressureAdvanceDrives : 4,		// which drivers have pressure advance applied
			 endStopsToCheck : 4,			// which drivers have endstop checks applied
			 stopAllDrivesOnEndstopHit : 1;	// whether to stop all drivers when one endstop is hit

	float initialSpeedFraction;
	float finalSpeedFraction;

	float initialX;						// needed only for delta movement
	float initialY;						// needed only for delta movement
	float finalX;						// needed only for delta movement
	float finalY;						// needed only for delta movement
	float zMovement;					// needed only for delta movement

	struct
	{
		int32_t steps;					// net steps moved
	} perDrive[MaxDriversPerCanSlave];

	void DebugPrint() const;
};


// Here are the layouts for standard message types, excluding the first dword.

struct CanMessageM140								// also does M104, M141
{
	uint16_t heaterNumber;
	uint16_t spare;
	float setPoint;
};

struct CanMessageM303
{
	uint16_t heaterNumber;
};

// This struct describes a possible parameter in a CAN message.
// An array of these describes all the possible parameters. The list is terminated by a zero entry.
struct ParamDescriptor
{
	// The type of a parameter. The lower 4 bits are the item size, except for string.
	enum ParamType : uint8_t
	{
		none = 0,
		uint32 = 0x04,
		int32 = 0x14,
		float_p = 0x24,
		uint16 = 0x02,
		int16 = 0x12,
		pwmFreq = 0x22,
		uint8 = 0x01,
		int8 = 0x11,
		char_p = 0x21,
		string = 0x10,
		reducedString = 0x20
	};

	char letter;
	ParamType type;

	size_t ItemSize() const { return (size_t)type & 0x0F; }
};

// Generic message
struct CanMessageGeneric
{
	uint32_t paramMap;
	uint8_t data[60];

	void DebugPrint(const ParamDescriptor *pt = nullptr) const;
};

// Parameter tables for various messages that use the generic format.
// It's a good idea to put 32-bit parameters earlier than 16-bit parameters, and 16-bit parameters earlier than 8-bit or string parameters, so that accesses are aligned.

#define UINT32_PARAM(_c) { _c, ParamDescriptor::uint32 }
#define INT32_PARAM(_c) { _c, ParamDescriptor::int32 }
#define FLOAT_PARAM(_c) { _c, ParamDescriptor::float_p }
#define UINT16_PARAM(_c) { _c, ParamDescriptor::uint16 }
#define INT16_PARAM(_c) { _c, ParamDescriptor::int16 }
#define PWM_FREQ_PARAM(_c) { _c, ParamDescriptor::pwmFreq }
#define UINT8_PARAM(_c) { _c, ParamDescriptor::uint8 }
#define INT8_PARAM(_c) { _c, ParamDescriptor::int8 }
#define CHAR_PARAM(_c) { _c, ParamDescriptor::char_p }
#define STRING_PARAM(_c) { _c, ParamDescriptor::string }
#define REDUCED_STRING_PARAM(_c) { _c, ParamDescriptor::reducedString }
#define END_PARAMS { 0 }

constexpr ParamDescriptor M308Params[] =
{
	FLOAT_PARAM('T'),
	FLOAT_PARAM('B'),
	FLOAT_PARAM('C'),
	FLOAT_PARAM('R'),
	INT8_PARAM('L'),
	INT8_PARAM('H'),
	UINT8_PARAM('F'),
	UINT8_PARAM('S'),
	UINT8_PARAM('W'),
	REDUCED_STRING_PARAM('Y'),
	REDUCED_STRING_PARAM('P'),
	CHAR_PARAM('K'),
	END_PARAMS
};

constexpr ParamDescriptor M950Params[] =
{
	PWM_FREQ_PARAM('Q'),
	INT16_PARAM('T'),
	UINT16_PARAM('F'),
	UINT16_PARAM('H'),
	UINT16_PARAM('P'),
	UINT16_PARAM('S'),
	REDUCED_STRING_PARAM('C'),
	END_PARAMS
};

constexpr ParamDescriptor M307Params[] =
{
	FLOAT_PARAM('A'),
	FLOAT_PARAM('C'),
	FLOAT_PARAM('D'),
	FLOAT_PARAM('S'),
	FLOAT_PARAM('V'),
	UINT16_PARAM('H'),
	UINT8_PARAM('B'),
	END_PARAMS
};

struct CanMessageM350
{
	uint32_t whichDrives;
	struct MicrostepSetting
	{
		uint32_t microstepping : 16,
				 interpolated : 1;
	};
	MicrostepSetting motorMicrostepping[MaxDriversPerCanSlave];
};

struct CanMessageM569
{
	uint32_t driverNumber;
	uint32_t driverMode : 3,
			 direction : 1,
			 enablePolarity : 1,
			 stepPulseLength : 5,
			 stepPulseInterval : 5,
			 dirSetupTime : 5,
			 dirHoldTime : 5,
			 tpfd : 4,
			 blankingTime : 2,
			 offTime : 4,
			 hstart : 3,
			 hend : 4,
			 hdec : 2,
			 tpwmthrs : 20,
			 thigh : 20;
};

struct CanMessageM906
{
	uint32_t whichDrives;
	struct MotorSetting
	{
		uint16_t currentMilliamps;
		uint8_t standstillCurrentPercent;
		uint8_t idleCurrentPercent;
	};
	MotorSetting motorCurrents[MaxDriversPerCanSlave];
};

struct CanMessageM913
{
	uint32_t whichDrives;
	uint32_t motorCurrentPercentages[MaxDriversPerCanSlave];
};

struct CanMessageM915
{
	uint32_t whichDrives;
	struct DriverStallSetting
	{
		//TODO
	};
	DriverStallSetting stallSettings[MaxDriversPerCanSlave];
};

union CanMessage
{
	CanMessageGeneric generic;
	CanMessageTimeSync sync;
	CanMessageEmergencyStop eStop;
	CanMessageMovement move;
	CanMessageM140 m140;
	CanMessageM303 m303;
	CanMessageM350 m350;
	CanMessageM569 m569;
	CanMessageM906 m906;
	CanMessageM913 m913;
	CanMessageM915 m915;
};

static_assert(sizeof(CanMessage) <= 64, "CAN message too big");		// check none of the messages is too large

// Standard responses

struct CanResponseHeaterStatus
{
	uint32_t status;						// 2 bits per heater: off, on, or fault
	struct HeaterStatus
	{
		int16_t actualTemp;
		float setPoint;
	};
	HeaterStatus heaterStatus[MaxHeatersPerCanSlave];
};

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
