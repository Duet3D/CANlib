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

// This message is used to set the following parameters for multiple drivers:
//  Motor currents: values are currents in mA
//  Microstepping:  values are microstepping (bits 0-8) and interpolation enable (bit 15)
//  Standstill current percentages:  values are the percentages
//  Driver states: 0 = disabled, 1 = idle, 2 = active
struct CanMessageMultipleDrivesRequest
{
	uint16_t driversToUpdate;
	uint16_t values[MaxDriversPerCanSlave];			// bits 0-10 are microstepping, but 15 is interpolation enable

	static constexpr uint16_t driverDisabled = 0, driverIdle = 1, driverActive = 2;

	size_t GetActualDataLength(size_t numDrivers) const { return sizeof(driversToUpdate) + numDrivers * sizeof(values[0]); }
};

struct CanMessageDiagnostics
{
	static constexpr CanMessageType messageType = CanMessageType::m122;

	uint8_t type;									// type of diagnostics requested, not currently used
};

struct CanMessageSetHeaterTemperature
{
	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heaterNumber;
	float setPoint;
};

struct CanMessageM303
{
	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heaterNumber;
	float targetTemperature;
};

struct CanMessageUpdateHeaterModel
{
	static constexpr CanMessageType messageType = CanMessageType::updateHeaterModel;

	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heater;
	float gain;
	float timeConstant;
	float deadTime;
	float maxPwm;
	float standardVoltage;					// power voltage reading at which tuning was done, or 0 if unknown
	bool enabled;
	bool usePid;
	bool inverted;
	bool pidParametersOverridden;

	// The next 3 are used only if pidParametersOverridden is true
	float kP;								// controller (not model) gain
	float recipTi;							// reciprocal of controller integral time
	float tD;								// controller differential time
};

struct __attribute__((packed)) CanTemperatureReport
{
	uint8_t errorCode;						// this holds a TemperatureError
	float temperature;						// the last temperature we read
};

struct __attribute__((packed)) CanMessageSensorTemperatures
{
	static constexpr CanMessageType messageType = CanMessageType::sensorTemperaturesReport;

	uint64_t whichSensors;					// which sensor numbers we have
	CanTemperatureReport temperatureReports[11];	// the error codes and temperatures of the ones we have, lowest sensor number first

	size_t GetActualDataLength(unsigned int numSensors) const { return numSensors * sizeof(CanTemperatureReport) + sizeof(uint64_t); }
};

struct CanMessageUpdateYourFirmware
{
	static constexpr CanMessageType messageType = CanMessageType::updateFirmware;
	uint8_t boardId;
	uint8_t invertedBoardId;
};

// This struct describes a possible parameter in a CAN message.
// An array of these describes all the possible parameters. The list is terminated by a zero entry.
struct ParamDescriptor
{
	// The type of a parameter. The lower 4 bits are the element size in bytes, except for string and reduced string.
	enum ParamType : uint8_t
	{
		length1 = 0x01,
		length2 = 0x02,
		length4 = 0x04,
		length8 = 0x08,
		isArray = 0x80,

		none = 0x00,
		uint64 = 0x00 | length8,
		uint32 = 0x00 | length4,
		uint16 = 0x00 | length2,
		uint8 = 0x00 | length1,

		uint32_array = uint32 | isArray,
		uint16_array = uint16 | isArray,
		uint8_array = uint8 | isArray,

		int32 = 0x10 | length4,
		int16 = 0x10 | length2,
		int8 = 0x10 | length1,
		string = 0x10,

#if 0	// these are not used or supported yet
		int32_array = int32 | isArray,
		int16_array = int16 | isArray,
		int8_array = int8 | isArray,
#endif

		float_p = 0x20 | length4,
		pwmFreq = 0x20 | length2,
		char_p = 0x20 | length1,
		reducedString = 0x20,

		localDriver = 0x40 | length1,

		float_array = float_p | isArray,
#if 0	// these are not used or supported yet
		pwmFreqArray = pwmFreq | isArray,
		char_array = char_p | isArray,
#endif
	};

	char letter;
	ParamType type;
	uint8_t maxArrayLength;

	size_t ItemSize() const { return (size_t)type & 0x0F; }		// only valid for some types
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
	uint32_t requestId : 12,
			 paramMap : 20;
	uint8_t data[60];

	void DebugPrint(const ParamDescriptor *pt = nullptr) const;

	static size_t GetActualDataLength(size_t paramLength) { return paramLength + sizeof(uint32_t); }
};

struct CanMessageStandardReply
{
	static constexpr CanMessageType messageType = CanMessageType::standardReply;

	uint16_t requestId : 12,
			 resultCode : 4;
	char text[62];

	static constexpr size_t MaxTextLength = sizeof(text);

	size_t GetTextLength(size_t dataLength) const
	{
		// can't use min<> here because it hasn't been moved to RRFLibraries yet
		return Strnlen(text, (dataLength < sizeof(uint16_t) + sizeof(text)) ? dataLength - sizeof(uint16_t) : sizeof(text));
	}

	size_t GetActualDataLength(size_t textLength) const
	{
		return textLength + sizeof(uint16_t);
	}
};

// Parameter tables for various messages that use the generic format.

#define UINT64_PARAM(_c) { _c, ParamDescriptor::uint64, 0 }
#define UINT32_PARAM(_c) { _c, ParamDescriptor::uint32, 0 }
#define UINT16_PARAM(_c) { _c, ParamDescriptor::uint16, 0 }
#define UINT8_PARAM(_c) { _c, ParamDescriptor::uint8, 0 }

#define INT32_PARAM(_c) { _c, ParamDescriptor::int32, 0 }
#define INT16_PARAM(_c) { _c, ParamDescriptor::int16, 0 }
#define INT8_PARAM(_c) { _c, ParamDescriptor::int8, 0 }

#define FLOAT_PARAM(_c) { _c, ParamDescriptor::float_p, 0 }
#define PWM_FREQ_PARAM(_c) { _c, ParamDescriptor::pwmFreq, 0 }
#define CHAR_PARAM(_c) { _c, ParamDescriptor::char_p, 0 }
#define STRING_PARAM(_c) { _c, ParamDescriptor::string, 0 }
#define REDUCED_STRING_PARAM(_c) { _c, ParamDescriptor::reducedString, 0 }
#define LOCAL_DRIVER_PARAM(_c) { _c, ParamDescriptor::localDriver, 0 }

#define UINT8_ARRAY_PARAM(_c, _n) { _c, ParamDescriptor::uint8_array, _n }
#define FLOAT_ARRAY_PARAM(_c, _n) { _c, ParamDescriptor::float_array, _n }

#define END_PARAMS { 0 }

constexpr ParamDescriptor M42Params[] =
{
	UINT16_PARAM('P'),
	FLOAT_PARAM('S'),
	END_PARAMS
};

constexpr ParamDescriptor M106Params[] =
{
	UINT16_PARAM('P'),
	FLOAT_PARAM('S'),
	FLOAT_PARAM('L'),
	FLOAT_PARAM('X'),
	FLOAT_PARAM('B'),
	UINT8_ARRAY_PARAM('H', 8),
	FLOAT_ARRAY_PARAM('T', 8),
	END_PARAMS
};

constexpr ParamDescriptor M280Params[] =
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

constexpr ParamDescriptor M569Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('S'),
	INT8_PARAM('R'),
	UINT8_PARAM('D'),
	UINT8_PARAM('F'),
	UINT8_PARAM('B'),
	UINT16_PARAM('H'),
	UINT16_PARAM('V'),
	UINT8_ARRAY_PARAM('Y', 3),
	FLOAT_ARRAY_PARAM('T', 4),
	END_PARAMS
};

constexpr ParamDescriptor M915Params[] =
{
	UINT16_PARAM('d'),					// this is the bitmap of driver numbers to change the parameters for
	INT8_PARAM('S'),
	UINT8_PARAM('F'),
	UINT16_PARAM('H'),
	UINT16_PARAM('T'),
	UINT8_PARAM('R'),
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
	CanMessageUpdateHeaterModel heaterModel;
	CanMessageMultipleDrivesRequest multipleDrivesRequest;
	CanMessageUpdateYourFirmware updateYourFirmware;
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
