/*
 * CanMessageFormats.h
 *
 *  Created on: 16 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEFORMATS_H_
#define SRC_CAN_CANMESSAGEFORMATS_H_

#include "CanId.h"
#include "RRF3Common.h"
#include "CanSettings.h"
#include "RemoteInputHandle.h"

#include <General/Bitmap.h>
#include <General/Strnlen.h>
#include <General/Portability.h>

#include <climits>
#include <ctime>
#include <cstring>

constexpr unsigned int MaxLinearDriversPerCanSlave = 8;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

size_t CanAdjustedLength(size_t rawLength) noexcept;

// CAN message formats
// Some messages end in strings. For such messages, it is now safe to computing the message length without allowing for a null terminator.
// This is because when our sending functions need to round up the message length to a supported CAN size, the additional data is now set to zeros.
// All fields named 'zero' are spare and should be set to 0 for compatibility with future uses

// Time sync message. The realTime field was added at RRF3.2 so it is not transmitted by main boards running 3.1.1 and earlier.
struct __attribute__((packed)) CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	uint32_t timeSent;								// when this message was sent
	uint32_t lastTimeSent;							// when we tried to send the previous message
	uint32_t lastTimeAcknowledgeDelay : 16,			// the delay from that time before the previous message was acknowledged
			 isPrinting : 1,						// set if we are printing and filament monitor should collect data
			 zero : 15;								// unused
	uint32_t realTime;								// seconds since 00:00:00 UTC on 1 January 1970, unsigned to avoid year 2038 problem. Not always present.

	static constexpr size_t SizeWithoutRealTime = 12;	// length of message that doesn't include real time
	static constexpr size_t SizeWithRealTime = 16;	// minimum length of message that includes real time
};

// Emergency stop message
struct __attribute__((packed)) CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;

	void SetRequestId(CanRequestId rid) noexcept { }			// these messages don't need RIDs
};

// Enter test mode message, used to force a main board to behave like a CAN expansion board
struct __attribute__((packed)) CanMessageEnterTestMode
{
	static constexpr CanMessageType messageType = CanMessageType::enterTestMode;

	uint16_t requestId : 12,
			 zero1 : 4;
	uint16_t address : 7,										// CAN address to use
			 zero2 : 9;											// reserved for future use
	uint32_t passwd;											// integrity check

	static constexpr uint32_t Passwd = 0x57a82fd1;				// value in password field that must match

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero1 = 0; zero2 = 0; }
};

// Announce acknowledgement message
struct __attribute__((packed)) CanMessageAcknowledgeAnnounce
{
	static constexpr CanMessageType messageType = CanMessageType::acknowledgeAnnounce;

	void SetRequestId(CanRequestId rid) noexcept { }			// these messages don't need RIDs
};

// Reset message
struct __attribute__((packed)) CanMessageReset
{
	static constexpr CanMessageType messageType = CanMessageType::reset;

	uint16_t requestId : 12,
			 zero : 4;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Stop movement on specific drivers
struct __attribute__((packed)) CanMessageStopMovement
{
	static constexpr CanMessageType messageType = CanMessageType::stopMovement;

	uint16_t whichDrives;							// 0xFFFF if all drives on board to be stopped

	void SetRequestId(CanRequestId rid) noexcept { }			// these messages don't need RIDs
};

// Revert position on specific drivers
struct __attribute__((packed)) CanMessageRevertPosition
{
	static constexpr CanMessageType messageType = CanMessageType::revertPosition;

	uint32_t whichDrives : 16,									// bitmap of driver numbers whose required step counts are included n the message
			 zero: 16;
	uint32_t clocksAllowed;										// how many step clocks we allow for the move
	int32_t finalStepCounts[MaxLinearDriversPerCanSlave];		// the net number of steps of the last move that were required

	void SetRequestId(CanRequestId rid) noexcept { zero = 0; }	// these messages don't need RIDs
	static constexpr size_t GetActualDataLength(size_t numReverting) noexcept { return (2 * sizeof(uint32_t)) + (numReverting * sizeof(int32_t)); }
};

static_assert(CanMessageRevertPosition::GetActualDataLength(MaxLinearDriversPerCanSlave) == sizeof(CanMessageRevertPosition));

// Movement messages

#if 0

struct __attribute__((packed)) CanMessageMovementLinear
{
	static constexpr CanMessageType messageType = CanMessageType::movementLinear;

	uint32_t whenToExecute;							// the master clock time at which this move should start
	uint32_t accelerationClocks;					// how many clocks the acceleration phase should last
	uint32_t steadyClocks;							// how many clocks the steady speed phase should last
	uint32_t decelClocks;							// how many clocks the deceleration phase should last

	uint32_t pressureAdvanceDrives : 8,				// which drivers have pressure advance applied
			 numDrivers : 4,						// how many drivers we included
			 seq : 7,								// sequence number
			 zero : 13;								// unused

	static constexpr uint8_t SeqMask = 0x7f;

	float initialSpeedFraction;						// the initial speed divided by the top speed
	float finalSpeedFraction;						// the final speed divided by the top speed

	struct PerDriveValues
	{
		int32_t steps;								// net steps moved by this drive

		void Init() noexcept
		{
			steps = 0;
		}
	};

	PerDriveValues perDrive[MaxLinearDriversPerCanSlave];

	void SetRequestId(CanRequestId rid) noexcept	// these messages don't have RIDs
	{
		zero = 0;
	}

	void DebugPrint() const noexcept;

	size_t GetActualDataLength() const noexcept
	{
		return (sizeof(*this) - sizeof(perDrive)) + (numDrivers * sizeof(perDrive[0]));
	}

	// This is called from just one place (in CanMotion::FinishMovement), so inline
	bool HasMotion() const noexcept
	{
		for (size_t drive = 0; drive < numDrivers; ++drive)
		{
			if (perDrive[drive].steps != 0)
			{
				return true;
			}
		}
		return false;
	}
};

#endif

struct __attribute__((packed)) CanMessageMovementLinearShaped
{
	static constexpr CanMessageType messageType = CanMessageType::movementLinearShaped;

	uint32_t whenToExecute;							// the master clock time at which this move should start
	uint32_t accelerationClocks;					// how many clocks the acceleration phase should last
	uint32_t steadyClocks;							// how many clocks the steady speed phase should last
	uint32_t decelClocks;							// how many clocks the deceleration phase should last

	uint32_t extruderDrives : 8,					// which drivers are for extruders
			 numDrivers : 4,						// how many drivers we included (maximum is 8)
			 seq : 4,								// sequence number
			 zero1 : 8,								// was used to hold the input shaping plan for this move
			 usePressureAdvance : 1,				// true to apply PA to the extruders and accumulate partial steps
			 useLateInputShaping : 1,
			 zero2 : 6;								// unused

	static constexpr uint8_t SeqMask = 0x0f;

	float acceleration;								// the base acceleration during the acceleration segment, when the total distance is normalised to 1.0
	float deceleration;								// the base deceleration during the deceleration segment, when the total distance is normalised to 1.0

	union PerDriveValues
	{
		int32_t steps;								// net steps moved by this drive (for non-extruders)
		float extrusion;							// how many steps of extrusion to do (for extruders) including fractional parts

		void Init() noexcept
		{
			steps = 0;
		}
	};

	PerDriveValues perDrive[MaxLinearDriversPerCanSlave];

	void SetRequestId(CanRequestId rid) noexcept	// these messages don't have RIDs
	{
		extruderDrives = 0;
		usePressureAdvance = 0;
		useLateInputShaping = 0;
		zero1 = zero2 = 0;
	}

	void DebugPrint() const noexcept;

	size_t GetActualDataLength() const noexcept
	{
		return (sizeof(*this) - sizeof(perDrive)) + (numDrivers * sizeof(perDrive[0]));
	}

	// This is called from just one place (in CanMotion::FinishMovement), so inline
	bool HasMotion() const noexcept
	{
		for (size_t drive = 0; drive < numDrivers; ++drive)
		{
			if (perDrive[drive].steps != 0)				// we rely on this being valid even if perDrive[drive] contains [positive] floating point zero
			{
				return true;
			}
		}
		return false;
	}
};

// Change CAN address and normal timing message
struct __attribute__((packed)) CanMessageSetAddressAndNormalTiming
{
	static constexpr CanMessageType messageType = CanMessageType::setAddressAndNormalTiming;

	uint16_t requestId : 12,
			 zero : 4;
	uint8_t oldAddress;
	uint8_t newAddress;
	uint8_t newAddressInverted;
	uint8_t doSetTiming;
	CanTiming normalTiming;

	static constexpr uint8_t DoSetTimingYes = 0xB6;				// magic byte to indicate that we do want to write the timing data
	static constexpr uint8_t DoSetTimingNo = 0;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// This message is used to set the following parameters for multiple drivers:
//  Motor currents: values are currents in mA
//  Microstepping:  values are microstepping (bits 0-8) and interpolation enable (bit 15)
//  Standstill current percentages:  values are the percentages
//  Driver states: 0 = disabled, 1 = idle, 2 = active
template<class T> struct __attribute__((packed)) CanMessageMultipleDrivesRequest
{
	uint16_t requestId : 12,
			 zero : 4;
	uint16_t driversToUpdate;
	T values[MaxLinearDriversPerCanSlave];

	size_t GetActualDataLength(size_t numDrivers) const noexcept { return sizeof(uint16_t) * 2 + numDrivers * sizeof(T); }
	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Type of data used to send microstepping and steps/mm data in a CanMessageMultipleDrivesRequest
struct __attribute__((packed)) StepsPerUnitAndMicrostepping
{
	float stepsPerUnit;
	uint16_t microstepping;

	StepsPerUnitAndMicrostepping(float spu, uint16_t ms) noexcept
	{
		StoreLEF32(&stepsPerUnit, spu);
		microstepping = ms;
	}

	StepsPerUnitAndMicrostepping() noexcept { }

	float GetStepsPerUnit() const noexcept
	{
		return LoadLEF32(&stepsPerUnit);
	}

	uint16_t GetMicrostepping() const noexcept
	{
		return microstepping;
	}
};

// Type of data used to send driver status
struct __attribute__((packed)) DriverStateControl
{
	// In the following the meaning of idlePercentOrDelayAfterBrakeOn is:
	// - If the mode is driverIdle then the top 8 bits are the idle current percent, to match the earlier version of this struct
	// - If the mode is driverDisabled then all 12 bits are the delay in milliseconds between re-engaging the brake and disabling the motor
	// - If the mode is driverEnabled then all 12 bits are the delay in milliseconds between enabling the motor and disengaging the brake
	uint16_t mode : 2,									// see value below
			 zero : 2,
			 idlePercentOrDelayAfterBrakeOn : 12;

	DriverStateControl() noexcept : mode(0), zero(0), idlePercentOrDelayAfterBrakeOn(0) { }
	DriverStateControl(uint16_t m, uint16_t idlePcOrBrakeDelay) noexcept : mode(m), zero(0), idlePercentOrDelayAfterBrakeOn(idlePcOrBrakeDelay) { }

	static constexpr uint16_t driverDisabled = 0, driverIdle = 1, driverActive = 2;		// values for 'mode'
};

struct __attribute__((packed)) CanMessageReturnInfo
{
	static constexpr CanMessageType messageType = CanMessageType::returnInfo;
	static constexpr uint8_t typeFirmwareVersion = 0;
	static constexpr uint8_t typeBoardName = 1;
	static constexpr uint8_t unused_was_typePressureAdvance = 2;
	static constexpr uint8_t typeM408 = 3;
	static constexpr uint8_t typeBootloaderName = 4;
	static constexpr uint8_t typeBoardUniqueId = 5;
	static constexpr uint8_t typeDiagnosticsPart0 = 100;
	// Other parts of the diagnostics reply use 101, 102 etc. so keep these free

	uint16_t requestId : 12,
			 param : 4;								// M408 S parameter or M122 P parameter
	uint8_t type;									// type of info requested

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; }
};

struct __attribute__((packed)) CanMessageDiagnosticTest
{
	static constexpr CanMessageType messageType = CanMessageType::diagnosticTest;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t testType;								// the M122 P parameter
	uint16_t invertedTestType;						// the complement of the M122 P parameter
	uint16_t param16;								// possible 16-bit parameter
	uint32_t param32[3];							// possible 32-bit parameters

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

struct __attribute__((packed)) CanMessageSetHeaterTemperature
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterTemperature;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heaterNumber : 8,
			 zero2 : 7,
			 isBedOrChamber : 1;
	float setPoint;
	uint8_t command : 4,
			zero3 : 4;

	static constexpr uint8_t commandNone = 0;
	static constexpr uint8_t commandOff = 1;
	static constexpr uint8_t commandOn = 2;
	static constexpr uint8_t commandResetFault = 3;
	static constexpr uint8_t commandSuspend = 4;
	static constexpr uint8_t commandUnsuspend = 5;
	static constexpr uint8_t commandReset = 6;				// reset the heater after a failed model update

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; zero3 = 0; }
};

struct __attribute__((packed)) CanMessageM303
{
	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heaterNumber;
	float targetTemperature;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

struct __attribute__((packed)) CanMessageHeaterModelNewNew
{
	static constexpr CanMessageType messageType = CanMessageType::heaterModelNewNew;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heater : 8,
			 enabled : 1,
			 usePid : 1,
			 inverted : 1,
			 pidParametersOverridden : 1,
			 zero2 : 4;
	float heatingRate;
	float basicCoolingRate;
	float fanCoolingRate;
	float coolingRateExponent;
	float fZero;							// earmarked for extra cooling rate due to extrusion
	float deadTime;
	float maxPwm;
	float standardVoltage;					// power voltage reading at which tuning was done, or 0 if unknown

	// The next 3 are used only if pidParametersOverridden is true
	float kP;								// controller (not model) gain
	float recipTi;							// reciprocal of controller integral time
	float tD;								// controller differential time

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// M570 parameters
// IMPORTANT! Field maxBadTemperatureCount was added at version 3.5.
// Boards receiving this message must not use the maxBadTemperatureReadings unless the version35 bit is set.
struct __attribute__((packed)) CanMessageSetHeaterFaultDetectionParameters
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterFaultDetection;

	uint16_t requestId : 12,
			 version35 : 1,
			  	  zero : 3;
	uint16_t heater;
	float maxTempExcursion;
	float maxFaultTime;
	uint32_t maxBadTemperatureCount;		// added at version 3.5; only present if the version35 flag is set

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; version35 = 0; }
};

struct __attribute__((packed)) CanMessageSetHeaterMonitors
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterMonitors;

	uint16_t requestId : 12,
			 numMonitors : 4;
	uint16_t heater;
	struct __attribute__((packed)) CanHeaterMonitor
	{
		float limit;
		int8_t sensor;
		uint8_t action;
		int8_t trigger;
		uint8_t zero;
	};
	CanHeaterMonitor monitors[7];

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; }

	size_t GetActualDatalength() const noexcept { return (2 * sizeof(uint16_t)) + (numMonitors * sizeof(CanHeaterMonitor)); }
};

struct __attribute__((packed)) CanMessageUpdateYourFirmware
{
	static constexpr CanMessageType messageType = CanMessageType::updateFirmware;

	uint16_t requestId : 12,
			 module : 2,					// 0 = main firmware, 1 = bootloader, 2,3 reserved
			 zero : 2;
	uint8_t boardId;
	uint8_t invertedBoardId;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

struct __attribute__((packed)) CanMessageFanParameters
{
	static constexpr CanMessageType messageType = CanMessageType::fanParameters;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t fanNumber;
	uint16_t blipTime;						// in milliseconds
	float val;
	float minVal;
	float maxVal;
	float triggerTemperatures[2];
	uint64_t sensorsMonitored;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

struct __attribute__((packed)) CanMessageSetFanSpeed
{
	static constexpr CanMessageType messageType = CanMessageType::setFanSpeed;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t fanNumber;
	float pwm;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

#if 0

// Request to create an input monitor
struct __attribute__((packed)) CanMessageCreateInputMonitorOld
{
	static constexpr CanMessageType messageType = CanMessageType::createInputMonitorOld;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	uint16_t threshold;			// analog threshold, or zero if digital
	uint16_t minInterval;
	char pinName[56];			// null terminated

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
	size_t GetActualDataLength() const noexcept { return 3 * sizeof(uint16_t) + sizeof(RemoteInputHandle) + Strnlen(pinName, sizeof(pinName)/sizeof(pinName[0])); }
	size_t GetMaxPinNameLength(size_t dataLength) const noexcept { return dataLength - (3 * sizeof(uint16_t) + sizeof(RemoteInputHandle)); }
};

// Request to reconfigure an input monitor
struct __attribute__((packed)) CanMessageChangeInputMonitorOld
{
	static constexpr CanMessageType messageType = CanMessageType::changeInputMonitorOld;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	uint16_t param;
	uint8_t action;

	static constexpr uint8_t actionDontMonitor = 0, actionDoMonitor = 1, actionDelete = 2, actionChangeThreshold = 3, actionChangeMinInterval = 4,
								actionReturnPinName = 5;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

#endif

// Request to create an input monitor
struct __attribute__((packed)) CanMessageCreateInputMonitorNew
{
	static constexpr CanMessageType messageType = CanMessageType::createInputMonitorNew;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	uint32_t threshold;			// analog threshold, or zero if digital
	uint16_t minInterval;
	char pinName[54];			// null terminated

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
	size_t GetActualDataLength() const noexcept { return 2 * sizeof(uint16_t) + sizeof(uint32_t) + sizeof(RemoteInputHandle) + Strnlen(pinName, sizeof(pinName)/sizeof(pinName[0])); }
	size_t GetMaxPinNameLength(size_t dataLength) const noexcept { return dataLength - (2 * sizeof(uint16_t) + sizeof(uint32_t) + sizeof(RemoteInputHandle)); }
};

// Request to reconfigure an input monitor
struct __attribute__((packed)) CanMessageChangeInputMonitorNew
{
	static constexpr CanMessageType messageType = CanMessageType::changeInputMonitorNew;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	uint32_t param;
	uint8_t action;

	static constexpr uint8_t actionDontMonitor = 0, actionDoMonitor = 1, actionDelete = 2, actionChangeThreshold = 3, actionChangeMinInterval = 4,
								actionReturnPinName = 5, actionSetDriveLevel = 6;

	// When the action is actionSetDriveLevel, some values of param define a special action:
	static constexpr uint32_t paramAutoCalibrateDriveLevelAndReport = 0xFFFFFFFF, paramReportDriveLevel = 0xFFFFFFFE;
	static constexpr uint32_t paramDriveLevelMask = 0x1F;			// bottom 5 bits are the drive level
	static constexpr unsigned int paramOffsetShift = 5;				// remaining bits are the offset
	static constexpr uint32_t maxParamOffset = ((uint32_t)1 << (32 - paramOffsetShift)) - 1;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Struct to represent an analog handle and the reading from it
struct __attribute__((packed)) AnalogHandleData
{
	RemoteInputHandle handle;
	uint32_t reading;						// note, this will not be aligned!
};

// Request to read inputs, including analog inputs
struct __attribute__((packed)) CanMessageReadInputsRequest
{
	static constexpr CanMessageType messageType = CanMessageType::readInputsRequest;

	uint32_t requestId : 12,				// the request ID of the message we are replying to
			 zero : 20;
	RemoteInputHandle mask;					// the mask we use when matching handles
	RemoteInputHandle pattern;				// the handle pattern to match

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Request to start sending accelerometer data
struct __attribute__((packed)) CanMessageStartAccelerometer
{
	static constexpr CanMessageType messageType = CanMessageType::startAccelerometer;

	uint16_t requestId : 12,
			 zero1 : 4;
	uint8_t  deviceNumber;
	uint8_t  axes : 3,						// bitmap of axes to collect
			 delayedStart : 1,				// true to delay starting until startTime
			 zero2 : 4;
	uint32_t numSamples;					// how many samples to collect
	uint32_t startTime;						// step timer ticks at which to start collecting, if delayedStart is set

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero1 = 0; zero2 = 0; }
};

// Request to start sending closed loop data
struct __attribute__((packed)) CanMessageStartClosedLoopDataCollection
{
	static constexpr CanMessageType messageType = CanMessageType::startClosedLoopDataCollection;

	uint16_t requestId : 12,
			 zero1 : 4;
	uint16_t rate;							// The sample rate at which to collect
	uint16_t filter;						// what variables to collect;
	uint8_t  deviceNumber;					// The device to collect data for
	uint8_t  mode;							// the mode to collect in
	uint16_t numSamples;					// how many samples to collect
	uint8_t  movement;						// Which (if any) movement was requested

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero1 = 0;}
};

// M42 or M280
struct __attribute__((packed)) CanMessageWriteGpio
{
	static constexpr CanMessageType messageType = CanMessageType::writeGpio;

	uint16_t requestId : 12,
			 isServo : 1,
			 zero : 3;
	float pwm;
	uint8_t portNumber;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0;}
};

// Create filament monitor (M591). We use a separate message to configure the filament monitor.
struct __attribute__((packed)) CanMessageCreateFilamentMonitor
{
	static constexpr CanMessageType messageType = CanMessageType::createFilamentMonitor;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t driver : 8,
			 zero2 : 4,
			 type : 8;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Delete a filament monitor (M591)
struct __attribute__((packed)) CanMessageDeleteFilamentMonitor
{
	static constexpr CanMessageType messageType = CanMessageType::deleteFilamentMonitor;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t driver : 8,
			 zero2 : 12;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Enter tuning mode (used by M303). This causes the heater to cycle between two temperatures, reporting data at the end of each cycle.
struct __attribute__((packed)) CanMessageHeaterTuningCommand
{
	static constexpr CanMessageType messageType = CanMessageType::heaterTuningCommand;

	uint16_t requestId : 12,
			 zero : 4;
	uint32_t heaterNumber : 8,
			 on : 1,
			 zero2 : 23;
	float pwm;
	float lowTemp;
	float highTemp;
	float peakTempDrop;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Configure heater feedforward
struct __attribute__((packed)) CanMessageHeaterFeedForward
{
	static constexpr CanMessageType messageType = CanMessageType::heaterFeedForward;

	uint16_t requestId : 12,
			 zero : 4;
	uint32_t heaterNumber : 8,
			 zero2 : 24;
	float fanPwmAdjustment;
	float extrusionAdjustment;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Configure input shaping
struct __attribute__((packed)) CanMessageSetInputShaping
{
	static constexpr CanMessageType messageType = CanMessageType::setInputShaping;

	struct ShapingPair { float coefficient; float delay; };

	uint16_t requestId : 12,
			 zero : 4;

	uint16_t numImpulses;								// the total number of impulses
	ShapingPair impulses[7];							// the coefficients and durations of the impulses

	size_t GetActualDataLength() const noexcept { return (2 * sizeof(uint16_t)) + (numImpulses * sizeof(ShapingPair)); }
	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Request to send a chunk of a firmware or bootloader file
struct __attribute__((packed)) CanMessageFirmwareUpdateRequest
{
	static constexpr CanMessageType messageType = CanMessageType::firmwareBlockRequest;

	uint32_t fileOffset : 24,			// the offset in the file of the data we need
			 bootloaderVersion: 5,		// the protocol version of the bootloader or firmware making this request, currently 0
			 uf2Format : 1,				// set if we want UF2 format, otherwise we want binary
			 fileWanted : 2;			// 0 = want firmware file, 1 and 2 reserved, 3 = want bootloader
	uint32_t lengthRequested : 24,		// how much data we want
			 boardVersion : 8;			// the hardware version of this board, currently always 0 for production boards
	char boardType[56];					// null-terminated board type name (firmware request) or bootloader class name (bootloader request)

	static constexpr uint32_t BootloaderVersion0 = 0;

	size_t GetActualDataLength() const noexcept { return 2 * sizeof(uint32_t) + Strnlen(boardType, sizeof(boardType)/sizeof(boardType[0])); }
	size_t GetBoardTypeLength(size_t dataLength) const noexcept { return dataLength - 2 * sizeof(uint32_t); }
	void SetRequestId(CanRequestId rid) noexcept { }		// these messages don't have RIDs
};

// Firmware update response
struct __attribute__((packed)) CanMessageFirmwareUpdateResponse
{
	static constexpr CanMessageType messageType = CanMessageType::firmwareBlockResponse;

	uint32_t fileOffset : 24,			// the offset in the file where this block starts
			 dataLength : 6,			// the number of bytes of data that follow
			 err : 2;					// the error code
	uint32_t fileLength : 24,			// the total size of the firmware file
			 zero : 8;
	uint8_t data[56];					// up to 56 bytes of data

	static constexpr uint32_t ErrNone = 0;
	static constexpr uint32_t ErrNoFile = 1;
	static constexpr uint32_t ErrBadOffset = 2;
	static constexpr uint32_t ErrOther = 3;

	size_t GetActualDataLength() const noexcept { return dataLength + 2 * sizeof(uint32_t); }
	void SetRequestId(CanRequestId rid) noexcept { zero = 0; }	// we don't have or need request IDs in this message type
};

// This is the standard reply used by many calls. It carries a GCodeResult, some text, and in some cases 8 bits of additional information.
// It can be split into multiple fragments so that the text is no constrained to 64 characters.
// The layout of requestId and resultCode are common to more than one reply type
struct __attribute__((packed)) CanMessageStandardReply
{
	static constexpr CanMessageType messageType = CanMessageType::standardReply;

	uint32_t requestId : 12,				// the request ID of the message we are replying to
			 resultCode : 4,				// normally a GCodeResult
			 fragmentNumber : 7,			// the fragment number of this message
			 moreFollows : 1,				// set if this is not the last fragment of the reply
			 extra : 8;						// normally unused, but occasionally carries extra data
	char text[60];

	static constexpr size_t MaxTextLength = sizeof(text);

	size_t GetTextLength(size_t dataLength) const noexcept
	{
		// can't use min<> here because it hasn't been moved to RRFLibraries yet
		return Strnlen(text, (dataLength < sizeof(uint32_t) + sizeof(text)) ? dataLength - sizeof(uint32_t) : sizeof(text));
	}

	size_t GetActualDataLength(size_t textLength) const noexcept
	{
		return textLength + sizeof(uint32_t);
	}

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; }
};

// Response to the ReadInputsRequest. The requestID and resultCode must be in the same place as for a standard reply.
struct __attribute__((packed)) CanMessageReadInputsReply
{
	static constexpr CanMessageType messageType = CanMessageType::readInputsReply;

	uint32_t requestId : 12,				// the request ID of the message we are replying to - must be in the same place as in a StandardReply
			 resultCode : 4,				// normally a GCodeResult - must be in the same place as in a StandardReply
			 numReported : 4,				// number of input handles reported
			 zero : 12;						// spare
	AnalogHandleData results[10];

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }

	size_t GetActualDataLength() noexcept
	{
		return sizeof(uint32_t) + numReported * sizeof(results[0]);
	}
};

// Generic message. These are always used in conjunction with a ParamTable that is know to both sender and receiver.
// The table lists the parameters, each one defined by the parameter letter and the type of parameter.
// The paramMap bitmap indicates which parameters are present in the data. They are provided in the same order as in the ParamTable.
struct __attribute__((packed)) CanMessageGeneric
{
	uint32_t requestId : 12,
			 paramMap : 20;
	uint8_t data[60];

	void DebugPrint(const struct ParamDescriptor *pt = nullptr) const noexcept;

	static size_t GetActualDataLength(size_t paramLength) noexcept { return paramLength + sizeof(uint32_t); }
	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; }
};

// Messages sent from expansion boards to main board, or broadcast
struct __attribute__((packed)) CanSensorReport
{
	uint8_t errorCode;						// this holds a TemperatureError

	float GetTemperature() const noexcept { return LoadLEF32(&temperature); }
	void SetTemperature(float t) noexcept { StoreLEF32(&temperature, t); }
private:									// make unaligned members private
	float temperature;						// the last temperature we read
};

// Message broadcast by expansion boards and the main board to provide sensor temperatures
struct __attribute__((packed)) CanMessageSensorTemperatures
{
	static constexpr CanMessageType messageType = CanMessageType::sensorTemperaturesReport;

	uint64_t whichSensors;					// which sensor numbers we have
	CanSensorReport temperatureReports[11];	// the error codes and temperatures of the ones we have, lowest sensor number first

	size_t GetActualDataLength(unsigned int numSensors) const noexcept { return numSensors * sizeof(CanSensorReport) + sizeof(uint64_t); }
};

// Struct used in CanMessageHeaterStatus
struct __attribute__((packed)) CanHeaterReport
{
	uint8_t mode;							// a HeaterMode value
	uint8_t averagePwm;						// scaled to 0-255 to save space

	float GetTemperature() const noexcept { return LoadLEF32(&temperature); }
	void SetTemperature(float t) noexcept { StoreLEF32(&temperature, t); }
private:									// make unaligned members private
	float temperature;						// the last temperature we read
};

// Message broadcast by expansion boards to send heater status to the main board
struct __attribute__((packed)) CanMessageHeatersStatus
{
	static constexpr CanMessageType messageType = CanMessageType::heatersStatusReport;

	uint64_t whichHeaters;					// which heater numbers we have
	CanHeaterReport reports[9];				// the status and temperatures of the ones we have, lowest sensor number first

	size_t GetActualDataLength(unsigned int numHeaters) const noexcept { return numHeaters * sizeof(CanHeaterReport) + sizeof(uint64_t); }
};

// Message used by expansion boards running firmware 3.4.0beta4 and earlier to announce their presence on the CAN bus to other boards
struct __attribute__((packed)) CanMessageAnnounceOld
{
	static constexpr CanMessageType messageType = CanMessageType::announceOld;

	uint32_t timeSinceStarted;				// how long since we started up
	uint32_t numDrivers: 8,					// the number of motor drivers on this board
			 zero : 24;						// for future expansion, set to zero
	char boardTypeAndFirmwareVersion[56];	// the type short name of this board followed by '|' and the firmware version

	void SetRequestId(CanRequestId rid) noexcept { zero = 0; }	// these messages don't need RIDs

	size_t GetActualDataLength() const noexcept
			{ return (2 * sizeof(uint32_t)) + Strnlen(boardTypeAndFirmwareVersion, sizeof(boardTypeAndFirmwareVersion)/sizeof(boardTypeAndFirmwareVersion[0])); }

	static size_t GetMaxTextLength(size_t dataLength) noexcept { return dataLength - (2 * sizeof(uint32_t)); }
};

// Message used by expansion boards running firmware 3.4.0beta5 and later to announce their presence on the CAN bus to other boards
struct __attribute__((packed)) CanMessageAnnounceNew
{
	static constexpr CanMessageType messageType = CanMessageType::announceNew;

	uint32_t timeSinceStarted;				// how long since we started up
	uint8_t uniqueId[16];					// the unique ID of this board
	uint8_t numDrivers: 4,					// the number of motor drivers on this board
			usesUf2Binary : 1,				// set if this board takes a main firmware binary in .uf2 format
			zero : 3;						// for future expansion, set to zero
	char boardTypeAndFirmwareVersion[43];	// the type short name of this board followed by '|' and the firmware version

	void SetRequestId(CanRequestId rid) noexcept { zero = 0; }	// these messages don't need RIDs

	size_t GetActualDataLength() const noexcept
			{ return sizeof(timeSinceStarted) + sizeof(uniqueId) + sizeof(uint8_t) + Strnlen(boardTypeAndFirmwareVersion, sizeof(boardTypeAndFirmwareVersion)/sizeof(boardTypeAndFirmwareVersion[0])); }

	static size_t GetMaxTextLength(size_t dataLength) noexcept { return dataLength - (sizeof(timeSinceStarted) + sizeof(uniqueId) + sizeof(uint8_t)); }
};

// Struct used within the fans report message
struct __attribute__((packed)) FanReport
{
	uint16_t actualPwm;						// actual PWM value, 0-65535
	int16_t rpm;							// tacho reading, or -1 if no tacho configured
};

// Message used to broadcast the status of fans
struct __attribute__((packed)) CanMessageFansReport
{
	static constexpr CanMessageType messageType = CanMessageType::fansReport;

	uint64_t whichFans;						// which fan numbers we are reporting
	FanReport fanReports[14];				// the actual PWM and RPM readings of the fans

	size_t GetActualDataLength(unsigned int numReported) const noexcept { return numReported * sizeof(fanReports[0]) + sizeof(uint64_t); }
};

// Message sent by an expansion board when one of its monitored inputs has changed state
struct __attribute__((packed)) CanMessageInputChangedNew
{
	static constexpr CanMessageType messageType = CanMessageType::inputStateChangedNew;

	uint16_t states;						// 1 bit per reported handle
	uint8_t numHandles;
	uint8_t zero;
	AnalogHandleData results[10];

	// Add an entry. 'states' and 'numHandles' must be cleared to zero before adding the first one. Return true if successful, false if message is full.
	bool AddEntry(uint16_t h, uint32_t val, bool state) noexcept
	{
		if (numHandles < sizeof(results)/sizeof(results[0]))
		{
			if (state)
			{
				states |= 1ul << numHandles;
			}
			results[numHandles].handle.Set(h);
			StoreLEU32(&results[numHandles].reading, val);
			++numHandles;
			return true;
		}
		return false;
	}

	size_t GetActualDataLength() const noexcept
	{
		return sizeof(states) + sizeof(numHandles) + sizeof(zero) + (numHandles * sizeof(results[0]));
	}
};

// Message sent by expansion boards to report their general health
struct __attribute__((packed)) CanMessageBoardStatus
{
	static constexpr CanMessageType messageType = CanMessageType::boardStatusReport;

	uint32_t hasVin : 1,
			 hasV12 : 1,
			 hasMcuTemp : 1,
			 hasAccelerometer : 1,
			 hasClosedLoop : 1,
			 hasInductiveSensor : 1,
			 zero : 10,							// reserved for future use
			 underVoltage : 1,
			 numAnalogHandles : 3,				// how many instances of AnaloghandleData we append
			 zero2 : 12;
	int32_t neverUsedRam;
	MinCurMax values[3];						// values of Vin, V12 and CPU temperature
	// After the last present MinCurMax value the data for some analog handles follows (max 4 if all of Vin/V12/mcuTemp are supported)

	void Clear() noexcept
	{
		hasVin = hasV12 = hasMcuTemp = underVoltage = hasAccelerometer = hasClosedLoop = hasInductiveSensor = false;
		zero = zero2 = numAnalogHandles = 0;
	}

	size_t GetAnalogHandlesOffset() const noexcept
	{
		const unsigned int numMinCurMaxValues = hasVin + hasV12 + hasMcuTemp;
		return 2 * sizeof(uint32_t) + numMinCurMaxValues * sizeof(values[0]);
	}

	size_t GetActualDataLength() const noexcept
	{
		return GetAnalogHandlesOffset() + numAnalogHandles * sizeof(AnalogHandleData);
	}
};

// Message sent by expansion boards to report the status of their drivers
struct __attribute__((packed)) CanMessageDriversStatus
{
	static constexpr CanMessageType messageType = CanMessageType::driversStatusReport;

	// Struct to represent driver status
	struct __attribute__((packed)) OpenLoopStatus
	{
		uint32_t status;
	};

	// Struct to represent driver status including closed loop data
	struct __attribute__((packed)) ClosedLoopStatus
	{
		uint32_t status;
		float16_t averageCurrentFraction;
		float16_t maxCurrentFraction;
		float16_t rmsPositionError;
		float16_t maxAbsPositionError;
	};

	uint16_t numDriversReported : 4,
			 hasClosedLoopData : 1,
			 zero : 11;
	uint16_t zero2;									// for alignment
	union
	{
		OpenLoopStatus openLoopData[15];			// status of each driver if not closed loop
		ClosedLoopStatus closedLoopData[5];			// status of each driver if closed loop
	};

	size_t GetActualDataLength() const noexcept
	{
		return 2 * sizeof(uint16_t) + numDriversReported * ((hasClosedLoopData) ? sizeof(closedLoopData[0]) : sizeof(openLoopData[0]));
	}

	void SetStandardFields(unsigned int numReported, bool closedLoop) noexcept
	{
		numDriversReported = numReported;
		hasClosedLoopData = closedLoop;
		zero = 0;
		zero2 = 0;
	}
};

// This has to be declared outside struct CanMessageFilamentMonitorsStatusNew to avoid having to include this file in FilamentMonitor.h
struct __attribute__((packed)) FilamentMonitorDataNew
{
	uint32_t calibrationLength : 24,	// length in mm over which we measured
			 status : 4,				// standard filament status
			 zero : 2,					// reserved for future use
			 hasLiveData : 1;			// true if the following fields are meaningful for this sensor
	int16_t avgPercentage;
	int16_t minPercentage;
	int16_t maxPercentage;
	int16_t lastPercentage;
};

// Message sent by expansion boards to report the status of their filament monitors
struct __attribute__((packed)) CanMessageFilamentMonitorsStatusNew
{
	static constexpr CanMessageType messageType = CanMessageType::filamentMonitorsStatusReportNew;

	uint32_t driversReported : 8,			// bitmap of driver numbers with associated filament monitors reported in this message
			 zero : 24;
	FilamentMonitorDataNew data[5];

	size_t GetActualDataLength() const noexcept
	{
		return sizeof(uint32_t) + (Bitmap<uint32_t>(driversReported).CountSetBits() * sizeof(data[0]));
	}

	void SetStandardFields(Bitmap<uint32_t> drivers) noexcept
	{
		driversReported = drivers.GetRaw();
		zero = 0;
	}
};

// Message used by expansion boards to report the results of one heater tuning cycle
struct __attribute__((packed)) CanMessageHeaterTuningReport
{
	static constexpr CanMessageType messageType = CanMessageType::heaterTuningReport;

	uint32_t heater : 8,
			 zero : 8,
			 cyclesDone : 16;
	uint32_t ton;
	uint32_t toff;
	uint32_t dlow;
	uint32_t dhigh;
	float heatingRate;
	float coolingRate;
	float voltage;

	void SetStandardFields(unsigned int heaterNumber) noexcept
	{
		heater = heaterNumber;
		zero = 0;
	}
};

// Message used to send accelerometer data from an expansion board to the master
struct __attribute__((packed)) CanMessageAccelerometerData
{
	static constexpr CanMessageType messageType = CanMessageType::accelerometerData;

	uint32_t actualSampleRate : 14,			// measured sample rate, or zero if not measured yet
			 numSamples : 6,				// number of samples in this buffer, each sample has data for each requested axis
			 overflowed : 1,				// true if the accelerometer detected overflow
			 axes : 3,						// which axes are returned in this data
			 bitsPerSampleMinusOne : 4,		// how many bits each sample takes up, minus one
			 lastPacket : 1,				// set if this is the last packet
			 zero : 3;
	uint16_t firstSampleNumber;				// the number of the first sample
	uint16_t data[29];

	// Get the actual amount of data. Must call SetAxesAndResolution first to set up bitsPerSampleMinusOne.
	size_t GetActualDataLength() const noexcept
	{
		const unsigned int numAxes = (axes & 1u) + ((axes >> 1) & 1u) + ((axes >> 2) & 1u);
		return sizeof(uint32_t) + sizeof(uint16_t) + ((numSamples * (bitsPerSampleMinusOne + 1) * numAxes + 15)/16) * sizeof(uint16_t);
	}

	// Set the resolution and axes bits, and return the maximum number of samples that one message can accommodate
	size_t SetAxesAndResolution(uint8_t p_axes, uint8_t bitsPerSample) noexcept
	{
		bitsPerSampleMinusOne = bitsPerSample - 1;
		axes = p_axes & 0x07;
		const unsigned int numAxes = (axes & 1u) + ((axes >> 1) & 1u) + ((axes >> 2) & 1u);
		return (numAxes * bitsPerSample == 0) ? 0xFFFF : (sizeof(data) * CHAR_BIT)/(numAxes * bitsPerSample);
	}
};

// Message used to send closed loop data from an expansion board to the master
struct __attribute__((packed)) CanMessageClosedLoopData
{
	static constexpr CanMessageType messageType = CanMessageType::closedLoopData;

	uint32_t numSamples : 5,				// number of samples in this data packet
			 lastPacket : 1,				// set if this is the last packet
			 filter : 16,					// which variables are present in the data packet
			 overflowed : 1,				// true if there was buffer overflow
			 badSample : 1,					// true if we had a bad sample (should not happen)
			 zero : 8;						// Currently unused
	uint32_t firstSampleNumber: 20,			// the number of the first sample
			 zero2: 12;						// Currently unused
	uint8_t  data[56];

	// Get the actual amount of data
	size_t GetActualDataLength(size_t numDataBytes) const noexcept
	{
		return 2 * sizeof(uint32_t) + numDataBytes;
	}

	// Get the number of data bytes in a message, given the message length (which will have been rounded up to the next CAN-FD value)
	static size_t GetNumDataBytes(size_t msglen) noexcept
	{
		return msglen - 2 * sizeof(uint32_t);
	}
};

// Message sent by an expansion board to the main board to indicate an event
struct __attribute__((packed)) CanMessageEvent
{
	static constexpr CanMessageType messageType = CanMessageType::event;

	uint32_t eventType : 8,		// the event type (what happened)
			deviceNumber : 8,	// the device number that it happened to
			eventParam : 16;	// more info about the event
	uint32_t zero;				// reserved for future use
	char text[56];				// other information about the event, to display to the user

	// Get the actual amount of data
	size_t GetActualDataLength() const noexcept
	{
		return 2 * sizeof(uint32_t) + Strnlen(text, ARRAY_SIZE(text));
	}

	// Get the maximum length of the text
	size_t GetMaxTextLength(size_t msgLen) const noexcept
	{
		return msgLen - 2 * sizeof(uint32_t);
	}
};

// Debug text message, sent by the expansion board to the main board
struct __attribute__((packed)) CanMessageDebugText
{
	static constexpr CanMessageType messageType = CanMessageType::debugText;

	char text[64];				// other information about the event, to display to the user

	// Get the actual amount of data
	size_t GetActualDataLength() const noexcept
	{
		return Strnlen(text, ARRAY_SIZE(text));
	}

	// Get the maximum length of the text
	size_t GetMaxTextLength(size_t msgLen) const noexcept
	{
		return msgLen;
	}
};

// A union of all message types to allow the correct message format to be extracted from a message buffer
union CanMessage
{
	CanMessage() noexcept { }

	uint8_t raw[64];
	uint32_t raw32[16];
	CanMessageGeneric generic;
	CanMessageTimeSync sync;
	CanMessageEmergencyStop eStop;
	CanMessageEnterTestMode enterTestMode;
	CanMessageStopMovement stopMovement;
	CanMessageRevertPosition revertPosition;
	CanMessageReset reset;
#if 0
	CanMessageMovementLinear moveLinear;
#endif
	CanMessageMovementLinearShaped moveLinearShaped;
	CanMessageReturnInfo getInfo;
	CanMessageSetHeaterTemperature setTemp;
	CanMessageStandardReply standardReply;
	CanMessageFirmwareUpdateRequest firmwareUpdateRequest;
	CanMessageFirmwareUpdateResponse firmwareUpdateResponse;
	CanMessageSensorTemperatures sensorTemperaturesBroadcast;
	CanMessageHeatersStatus heatersStatusBroadcast;
	CanMessageHeaterModelNewNew heaterModelNewNew;
	CanMessageMultipleDrivesRequest<uint16_t> multipleDrivesRequestUint16;
	CanMessageMultipleDrivesRequest<float> multipleDrivesRequestFloat;
	CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping> multipleDrivesStepsPerUnitAndMicrostepping;
	CanMessageMultipleDrivesRequest<DriverStateControl> multipleDrivesRequestDriverState;
	CanMessageUpdateYourFirmware updateYourFirmware;
	CanMessageFanParameters fanParameters;
	CanMessageSetFanSpeed setFanSpeed;
	CanMessageSetHeaterFaultDetectionParameters setHeaterFaultDetection;
	CanMessageSetHeaterMonitors setHeaterMonitors;
	CanMessageSetInputShaping setInputShaping;
	CanMessageCreateInputMonitorNew createInputMonitorNew;
	CanMessageChangeInputMonitorNew changeInputMonitorNew;
	CanMessageInputChangedNew inputChangedNew;
	CanMessageFansReport fansReport;
	CanMessageWriteGpio writeGpio;
	CanMessageSetAddressAndNormalTiming setAddressAndNormalTiming;
	CanMessageAnnounceOld announceOld;
	CanMessageAnnounceNew announceNew;
	CanMessageAcknowledgeAnnounce acknowledgeAnnounce;
	CanMessageDiagnosticTest diagnosticTest;
	CanMessageReadInputsRequest readInputsRequest;
	CanMessageReadInputsReply readInputsReply;
	CanMessageBoardStatus boardStatus;
	CanMessageDriversStatus driversStatus;
	CanMessageFilamentMonitorsStatusNew filamentMonitorsStatusNew;
	CanMessageCreateFilamentMonitor createFilamentMonitor;
	CanMessageDeleteFilamentMonitor deleteFilamentMonitor;
	CanMessageHeaterTuningCommand heaterTuningCommand;
	CanMessageHeaterTuningReport heaterTuningReport;
	CanMessageHeaterFeedForward heaterFeedForward;
	CanMessageStartAccelerometer startAccelerometer;
	CanMessageAccelerometerData accelerometerData;
	CanMessageStartClosedLoopDataCollection startClosedLoopDataCollection;
	CanMessageClosedLoopData closedLoopData;
	CanMessageEvent event;
	CanMessageDebugText debugText;
};

static_assert(sizeof(CanMessage) <= 64, "CAN message too big");		// check none of the messages is too large

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
