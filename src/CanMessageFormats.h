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
#include "General/Strnlen.h"

constexpr unsigned int MaxDriversPerCanSlave = 4;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

constexpr size_t MaxCanReplyLength = 63;				// 63 chars max, first byte is the GCodeResult

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


// Here are the layouts for standard message types

struct CanMessageSetHeaterTemperature
{
	float setPoint;
	uint16_t heaterNumber;
};

struct CanMessageM303
{
	uint16_t heaterNumber;
	float targetTemperature;
};

struct __attribute__((packed)) CanTemperatureReport
{
	uint8_t errorCode;						// this holds a TemperatureError
	float temperature;						// the last temperature we read
};

struct CanMessageSensorTemperatures
{
	static constexpr CanMessageType messageType = CanMessageType::sensorTemperaturesReport;

	uint64_t whichSensors;						// which sensor numbers we have
	CanTemperatureReport temperatureReports[11];	// the error codes and temperatures of the ones we have, lowest sensor number first
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

// Firmware update request
struct CanMessageFirmwareUpdateRequest
{
	static constexpr CanMessageType messageType = CanMessageType::FirmwareBlockRequest;

	uint32_t fileOffset : 24,			// the offset in the file of the data we need
			 bootloaderVersion: 8;		// the version of this bootloader
	uint32_t lengthRequested : 24,		// how much data we want
			 boardVersion : 8;			// the hardware version of this board
	char boardType[56];					// null-terminated board type name

	static constexpr uint32_t BootloaderVersion0 = 0;

	size_t GetActualDataLength() const { return 2 * sizeof(uint32_t) + Strnlen(boardType, sizeof(boardType)/sizeof(boardType[0])); }
	size_t GetBoardTypeLength(size_t dataLength) const { return dataLength - 2 * sizeof(uint32_t); }
};

// Firmware update response
struct CanMessageFirmwareUpdateResponse
{
	static constexpr CanMessageType messageType = CanMessageType::FirmwareBlockResponse;

	uint32_t fileOffset : 24,			// the offset in the file where this block starts
			 dataLength : 6,			// the number of bytes of data that follow
			 err : 2;					// the error code
	uint32_t fileLength : 24,			// the total size of the firmware file
			 spare2 : 8;
	uint8_t data[56];					// up to 56 bytes of data

	static constexpr uint32_t ErrNone = 0;
	static constexpr uint32_t ErrNoFile = 1;
	static constexpr uint32_t ErrBadOffset = 2;
	static constexpr uint32_t ErrOther = 3;

	size_t GetActualDataLength() const { return dataLength + 2 * sizeof(uint32_t); }
};

// Generic message
struct CanMessageGeneric
{
	uint32_t paramMap;
	uint8_t data[60];

	void DebugPrint(const ParamDescriptor *pt = nullptr) const;
};

struct CanMessageStandardReply
{
	static constexpr CanMessageType messageType = CanMessageType::standardReply;

	uint16_t requestId : 13,
			 resultCode : 3;
	char text[62];
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

constexpr ParamDescriptor M42Params[] =
{
	UINT16_PARAM('P'),
	FLOAT_PARAM('S'),
	END_PARAMS
};

constexpr ParamDescriptor M106Params[] =
{
	FLOAT_PARAM('S'),
	FLOAT_PARAM('L'),
	FLOAT_PARAM('X'),
	FLOAT_PARAM('B'),
//	FLOAT_ARRAY_PARAM('T'),
//	UINT16_ARRAY_PARAM('H'),
	UINT16_PARAM('P'),
	END_PARAMS
};

constexpr ParamDescriptor M208Params[] =
{
	UINT16_PARAM('P'),
	UINT16_PARAM('S'),
	END_PARAMS
};

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

union CanMessage
{
	uint8_t raw[64];
	CanMessageGeneric generic;
	CanMessageTimeSync sync;
	CanMessageEmergencyStop eStop;
	CanMessageMovement move;
	CanMessageSetHeaterTemperature setTemp;
	CanMessageStandardReply standardReply;
	CanMessageFirmwareUpdateRequest firmwareUpdateRequest;
	CanMessageFirmwareUpdateResponse firmwareUpdateResponse;
	CanMessageSensorTemperatures sensorTemperaturesBroadcast;
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
