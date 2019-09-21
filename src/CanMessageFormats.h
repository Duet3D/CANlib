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

size_t CanAdjustedLength(size_t rawLength);

// Type used to represent a handle to a remote input
struct __attribute__((packed)) RemoteInputHandle
{
	void Set(uint8_t p_type, uint8_t p_major, uint8_t p_minor) { u.parts.type = p_type; u.parts.major = p_major; u.parts.minor = p_minor; }
	void Set(uint16_t p_all) { u.all = p_all; }
	uint16_t asU16() const { return u.all; }

	union
	{
		struct
		{
			uint16_t minor : 8,						// endstop switch number within axis (for endstops), or trigger handle (for triggers)
					major : 4,						// axis number (for endstops)
					type : 4;
		} parts;
		uint16_t all;
	} u;

	static constexpr uint16_t typeEndstop = 0, typeTrigger = 1;
};

// CAN message formats

// Time sync message
struct __attribute__((packed)) CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	uint32_t timeSent;								// when this message was sent
	uint32_t lastTimeSent;							// when we tried to send the previous message
	uint32_t lastTimeAcknowledged;					// when the previous message was acknowledged
};

// Emergency stop message
struct __attribute__((packed)) CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;
};

// Stop movement on specific drives or all drives
struct __attribute__((packed)) CanMessageStopMovement
{
	static constexpr CanMessageType messageType = CanMessageType::stopMovement;

	uint16_t whichDrives;							// 0xFFFF if all drives on board to be stopped

	void SetRequestId(CanRequestId rid) { }			// these messages don't need RIDs
};

// Movement message
struct __attribute__((packed)) CanMessageMovement
{
	static constexpr CanMessageType messageType = CanMessageType::movement;

	uint32_t whenToExecute;
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;

	uint32_t deltaDrives : 4,						// which drivers are doing delta movement
			 pressureAdvanceDrives : 4,				// which drivers have pressure advance applied
			 endStopsToCheck : 4,					// which drivers have endstop checks applied
			 stopAllDrivesOnEndstopHit : 1;			// whether to stop all drivers when one endstop is hit

	float initialSpeedFraction;
	float finalSpeedFraction;

	float initialX;									// needed only for delta movement
	float initialY;									// needed only for delta movement
	float finalX;									// needed only for delta movement
	float finalY;									// needed only for delta movement
	float zMovement;								// needed only for delta movement

	struct
	{
		int32_t steps;								// net steps moved
	} perDrive[MaxDriversPerCanSlave];

	void SetRequestId(CanRequestId rid) { }			// these messages don't have RIDs, use the whenToExecute field to avoid duplication
	void DebugPrint() const;
};

// This message is used to set the following parameters for multiple drivers:
//  Motor currents: values are currents in mA
//  Microstepping:  values are microstepping (bits 0-8) and interpolation enable (bit 15)
//  Standstill current percentages:  values are the percentages
//  Driver states: 0 = disabled, 1 = idle, 2 = active
struct __attribute__((packed)) CanMessageMultipleDrivesRequest
{
	uint16_t requestId : 12,
			 spare : 4;
	uint16_t driversToUpdate;
	uint16_t values[MaxDriversPerCanSlave];			// bits 0-10 are microstepping, but 15 is interpolation enable

	static constexpr uint16_t driverDisabled = 0, driverIdle = 1, driverActive = 2;

	size_t GetActualDataLength(size_t numDrivers) const { return sizeof(uint16_t) * 2 + numDrivers * sizeof(values[0]); }
	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageReturnInfo
{
	static constexpr CanMessageType messageType = CanMessageType::returnInfo;
	static constexpr uint8_t typeFirmwareVersion = 0;
	static constexpr uint8_t typeBoardName = 1;
	static constexpr uint8_t typePressureAdvance = 2;
	static constexpr uint8_t typeDiagnosticsPart0 = 100;
	static constexpr uint8_t typeDiagnosticsPart1 = 101;
	static constexpr uint8_t typeDiagnosticsPart2 = 102;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t type;									// type of diagnostics requested, not currently used

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageSetHeaterTemperature
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterTemperature;

	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heaterNumber;
	float setPoint;
	uint8_t command;

	static constexpr uint8_t commandNone = 0;
	static constexpr uint8_t commandOff = 1;
	static constexpr uint8_t commandOn = 2;
	static constexpr uint8_t commandResetFault = 3;
	static constexpr uint8_t commandSuspend = 4;
	static constexpr uint8_t commandUnsuspend = 5;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageM303
{
	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heaterNumber;
	float targetTemperature;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageUpdateHeaterModel
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

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageSetHeaterFaultDetectionParameters
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterFaultDetection;

	uint16_t requestId : 12,
			 spare : 4;
	uint16_t heater;
	float maxTempExcursion;
	float maxFaultTime;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageUpdateYourFirmware
{
	static constexpr CanMessageType messageType = CanMessageType::updateFirmware;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t boardId;
	uint8_t invertedBoardId;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageFanParameters
{
	static constexpr CanMessageType messageType = CanMessageType::fanParameters;

	uint16_t requestId : 12,
			 spare : 4;
	uint16_t fanNumber;
	uint16_t blipTime;						// in milliseconds
	float val;
	float minVal;
	float maxVal;
	float triggerTemperatures[2];
	uint64_t sensorsMonitored;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageSetFanSpeed
{
	static constexpr CanMessageType messageType = CanMessageType::setFanSpeed;

	uint16_t requestId : 12,
			 spare : 4;
	uint16_t fanNumber;
	float pwm;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageCreateZProbe
{
	static constexpr CanMessageType messageType = CanMessageType::createZProbe;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t probeNumber;
	uint8_t probeType;
	char pinNames[60];				// actually a null-terminated string

	void SetRequestId(CanRequestId rid) { requestId = rid; }
	size_t GetActualDataLength() const { return CanAdjustedLength(sizeof(uint16_t) + 2 * sizeof(uint8_t) + Strnlen(pinNames, sizeof(pinNames)/sizeof(pinNames[0]))); }
	size_t GetMaxPinNamesLength(size_t dataLength) const { return dataLength - (sizeof(uint16_t) + 2 * sizeof(uint8_t)); }
};

struct __attribute__((packed)) CanMessageConfigureZProbe
{
	static constexpr CanMessageType messageType = CanMessageType::configureZProbe;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t number;
	uint8_t type;
	int16_t adcValue;				// the target ADC value, after inversion if enabled
	uint8_t invertReading;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageGetZProbePinNames
{
	static constexpr CanMessageType messageType = CanMessageType::getZProbePinNames;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t number;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageDestroyZProbe
{
	static constexpr CanMessageType messageType = CanMessageType::destroyZProbe;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t number;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageSetProbing
{
	static constexpr CanMessageType messageType = CanMessageType::setProbing;

	uint16_t requestId : 12,
			 spare : 4;
	uint8_t number;
	uint8_t isProbing;

	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct __attribute__((packed)) CanMessageCreateInputMonitor
{
	static constexpr CanMessageType messageType = CanMessageType::createInputMonitor;

	uint16_t requestId : 12,
			 spare : 4;
	RemoteInputHandle handle;
	uint16_t threshold;			// analog threshold, or zero if digital
	uint16_t minInterval;
	char pinName[56];			// null terminated

	void SetRequestId(CanRequestId rid) { requestId = rid; }
	size_t GetActualDataLength() const { return CanAdjustedLength(3 * sizeof(uint16_t) + sizeof(RemoteInputHandle) + Strnlen(pinName, sizeof(pinName)/sizeof(pinName[0]))); }
	size_t GetMaxPinNameLength(size_t dataLength) const { return dataLength - (3 * sizeof(uint16_t) + sizeof(RemoteInputHandle)); }
};

struct __attribute__((packed)) CanMessageChangeInputMonitor
{
	static constexpr CanMessageType messageType = CanMessageType::changeInputMonitor;

	uint16_t requestId : 12,
			 spare : 4;
	RemoteInputHandle handle;
	uint16_t param;
	uint8_t action;

	static constexpr uint8_t actionDontMonitor = 0, actionDoMonitor = 1, actionDelete = 2, actionChangeThreshold = 3, actionChangeMinInterval = 4;
	void SetRequestId(CanRequestId rid) { requestId = rid; }
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
	void SetRequestId(CanRequestId rid) { }		// these messages don't have RIDs
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
	void SetRequestId(CanRequestId rid) { }	// we don't have or need request IDs in this message type
};

// Generic message
struct CanMessageGeneric
{
	uint32_t requestId : 12,
			 paramMap : 20;
	uint8_t data[60];

	void DebugPrint(const ParamDescriptor *pt = nullptr) const;

	static size_t GetActualDataLength(size_t paramLength) { return paramLength + sizeof(uint32_t); }
	void SetRequestId(CanRequestId rid) { requestId = rid; }
};

struct CanMessageStandardReply
{
	static constexpr CanMessageType messageType = CanMessageType::standardReply;

	uint32_t requestId : 12,				// the request ID of the message we are replying to
			 resultCode : 4,				// normally a GCodeResult
			 fragmentNumber : 7,			// the fragment number of this message
			 moreFollows : 1,				// set if this is not the last fragment of the reply
			 extra : 8;						// normally unused, but occasionally carries extra data
	char text[60];

	static constexpr size_t MaxTextLength = sizeof(text);

	size_t GetTextLength(size_t dataLength) const
	{
		// can't use min<> here because it hasn't been moved to RRFLibraries yet
		return Strnlen(text, (dataLength < sizeof(uint32_t) + sizeof(text)) ? dataLength - sizeof(uint32_t) : sizeof(text));
	}

	size_t GetActualDataLength(size_t textLength) const
	{
		return textLength + sizeof(uint32_t);
	}

	void SetRequestId(CanRequestId rid) { requestId = rid; }
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

constexpr ParamDescriptor M950HeaterParams[] =
{
	UINT16_PARAM('H'),
	PWM_FREQ_PARAM('Q'),
	UINT16_PARAM('T'),
	REDUCED_STRING_PARAM('C'),
	END_PARAMS
};

constexpr ParamDescriptor M950FanParams[] =
{
	UINT16_PARAM('F'),
	PWM_FREQ_PARAM('Q'),
	STRING_PARAM('C'),
	END_PARAMS
};

constexpr ParamDescriptor M950GpioParams[] =
{
	UINT16_PARAM('P'),
	PWM_FREQ_PARAM('Q'),
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

// Messages sent from expansion boards to main board, or broadcast
struct __attribute__((packed)) CanSensorReport
{
	uint8_t errorCode;						// this holds a TemperatureError
	float temperature;						// the last temperature we read
};

struct __attribute__((packed)) CanMessageSensorTemperatures
{
	static constexpr CanMessageType messageType = CanMessageType::sensorTemperaturesReport;

	uint64_t whichSensors;					// which sensor numbers we have
	CanSensorReport temperatureReports[11];	// the error codes and temperatures of the ones we have, lowest sensor number first

	size_t GetActualDataLength(unsigned int numSensors) const { return numSensors * sizeof(CanSensorReport) + sizeof(uint64_t); }
};

struct __attribute__((packed)) CanHeaterReport
{
	uint8_t mode;							// a HeaterMode value
	uint8_t averagePwm;						// scaled to 0-255 to save space
	float temperature;						// the last temperature we read
};

struct __attribute__((packed)) CanMessageHeatersStatus
{
	static constexpr CanMessageType messageType = CanMessageType::heatersStatusReport;

	uint64_t whichHeaters;					// which heater numbers we have
	CanHeaterReport reports[9];				// the status and temperatures of the ones we have, lowest sensor number first

	size_t GetActualDataLength(unsigned int numHeaters) const { return numHeaters * sizeof(CanHeaterReport) + sizeof(uint64_t); }
};

struct __attribute__((packed)) CanMessageInputChanged
{
	static constexpr CanMessageType messageType = CanMessageType::inputStateChanged;

	uint32_t states;						// 1 bit per reported handle
	uint8_t numHandles;
	uint8_t spare;
	RemoteInputHandle handles[29];			// the handles reported

	void SetRequestId(CanRequestId rid) { }	// we don't have or need request IDs in this message type

	// Add an entry. 'states' and 'numHandles' must be cleared to zero before adding the first one.
	bool AddEntry(uint16_t h, bool state)
	{
		if (numHandles < sizeof(handles)/sizeof(handles[0]))
		{
			if (state)
			{
				states |= 1ul << numHandles;
			}
			handles[numHandles].Set(h);
			++numHandles;
			return true;
		}
		return false;
	}

	size_t GetActualDataLength() const
	{
		return sizeof(states) + sizeof(numHandles) + sizeof(spare) + numHandles * sizeof(handles[0]);
	}
};

union CanMessage
{
	uint8_t raw[64];
	CanMessageGeneric generic;
	CanMessageTimeSync sync;
	CanMessageEmergencyStop eStop;
	CanMessageStopMovement stopMovement;
	CanMessageMovement move;
	CanMessageReturnInfo getInfo;
	CanMessageSetHeaterTemperature setTemp;
	CanMessageStandardReply standardReply;
	CanMessageFirmwareUpdateRequest firmwareUpdateRequest;
	CanMessageFirmwareUpdateResponse firmwareUpdateResponse;
	CanMessageSensorTemperatures sensorTemperaturesBroadcast;
	CanMessageHeatersStatus heatersStatusBroadcast;
	CanMessageUpdateHeaterModel heaterModel;
	CanMessageMultipleDrivesRequest multipleDrivesRequest;
	CanMessageUpdateYourFirmware updateYourFirmware;
	CanMessageFanParameters fanParameters;
	CanMessageSetFanSpeed setFanSpeed;
	CanMessageSetHeaterFaultDetectionParameters setHeaterFaultDetection;
	CanMessageCreateZProbe createZProbe;
	CanMessageConfigureZProbe configureZProbe;
	CanMessageGetZProbePinNames getZProbePinNames;
	CanMessageDestroyZProbe destroyZProbe;
	CanMessageSetProbing setProbing;
	CanMessageCreateInputMonitor createInputMonitor;
	CanMessageChangeInputMonitor changeInputMonitor;
	CanMessageInputChanged inputChanged;
};

static_assert(sizeof(CanMessage) <= 64, "CAN message too big");		// check none of the messages is too large

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
