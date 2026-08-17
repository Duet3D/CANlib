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
#include "Duet3Common.h"
#include "CanSettings.h"
#include "RemoteInputHandle.h"
#include "HeaterModel.h"

#include <General/Bitmap.h>
#include <General/Strnlen.h>
#include <General/Portability.h>

#include <climits>
#include <ctime>
#include <cstring>

size_t CanAdjustedLength(size_t rawLength) noexcept;

// CAN message formats
// Some messages end in strings. For such messages, it is now safe to compute the message length without allowing for a null terminator.
// This is because when our sending functions need to round up the message length to a supported CAN size, the additional data is now set to zeros.
// All fields named 'zero' are spare and should be set to 0 for compatibility with future uses
// Message formats that take a request ID must have a method SetRequestId that sets the request ID and clears the zero fields
// Message formats that don't take a request ID must have a method ClearReservedFields that clears the zero fields

// Time sync message. The realTime field was added at RRF3.2 so it is not transmitted by main boards running 3.1.1 and earlier.
// Note, time sync messages are always transmitted without using BRS. We should look at making this message smaller,
// for example by re-ordering the fields so that we can send the movement delay without also sending real time.
struct __attribute__((packed)) CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	uint32_t timeSent;								// when this message was sent
	uint32_t lastTimeSent;							// when we tried to send the previous message
	uint32_t lastTimeAcknowledgeDelay : 16,			// the delay from that time before the previous message was acknowledged
			 isPrinting : 1,						// set if we are printing and filament monitor should collect data
			 fastDataRate : 3,						// CAN-FD data bit rate divided by nominal bit rate, minus 1. 0 (= multiplier 1) means don't use bit rate switching.
			 tseg1Minus1 : 8,						// the tseg1 value for the data phase minus 1
			 zero: 4;								// unused
	uint32_t realTime;								// seconds since 00:00:00 UTC on 1 January 1970, unsigned to avoid year 2038 problem. Not always present.
	uint32_t movementDelay;							// cumulative hiccup time. Not always present.

	static constexpr size_t SizeWithoutRealTime = 12;	// length of message that doesn't include real time
	static constexpr size_t SizeWithRealTime = 16;		// minimum length of message that includes real time
	static constexpr size_t SizeWithRealTimeAndMovementDelay = 20;	// length of message that includes real time and movement delay

	void ClearReservedFields() noexcept { zero = 0; }
};

// Emergency stop message
struct __attribute__((packed)) CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;

	void ClearReservedFields() noexcept { }
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

	void ClearReservedFields() noexcept { }
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

	void ClearReservedFields() noexcept { }
};

// Revert position on specific drivers
struct __attribute__((packed)) CanMessageRevertPosition
{
	static constexpr CanMessageType messageType = CanMessageType::revertPosition;

	uint32_t whichDrives : 16,									// bitmap of driver numbers whose required step counts are included n the message
			 zero: 16;
	uint32_t clocksAllowed;										// how many step clocks we allow for the move
	int32_t finalStepCounts[MaxLinearDriversPerCanSlave];		// the net number of steps of the last move that were required

	void ClearReservedFields() noexcept { zero = 0; }
	static constexpr size_t GetActualDataLength(size_t numReverting) noexcept { return (2 * sizeof(uint32_t)) + (numReverting * sizeof(int32_t)); }
};

static_assert(CanMessageRevertPosition::GetActualDataLength(MaxLinearDriversPerCanSlave) == sizeof(CanMessageRevertPosition));

// Movement messages

struct __attribute__((packed)) CanMessageMovementLinearShaped final
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

	float acceleration;								// the base acceleration during the acceleration segment, when the total distance is normalised to 1.0. Always positive or zero.
	float deceleration;								// the negative of the base deceleration during the deceleration segment, when the total distance is normalised to 1.0. Always positive or zero.

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

	void ClearReservedFields() noexcept
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

	static constexpr size_t GetActualDataLength(size_t numDrivers) noexcept { return sizeof(uint16_t) * 2 + numDrivers * sizeof(T); }
	static constexpr size_t MaxDrivesPerMessage() noexcept { return (64 - 2 * sizeof(uint16_t))/sizeof(T); }
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
			 zero : 6,
			 idlePercent : 8;

	DriverStateControl() noexcept : mode(0), zero(0), idlePercent(0) { }
	explicit DriverStateControl(uint16_t m, uint16_t idlePc = 0) noexcept : mode(m), zero(0), idlePercent(idlePc) { }

	static constexpr uint16_t driverDisabled = 0, driverIdle = 1, driverActive = 2;		// values for 'mode'
};

struct __attribute__((packed)) CanMessageReturnInfo
{
	static constexpr CanMessageType messageType = CanMessageType::returnInfo;
	static constexpr uint8_t typeFirmwareVersion = 0;
	static constexpr uint8_t typeBoardName = 1;
	static constexpr uint8_t unused_was_typePressureAdvance = 2;
	static constexpr uint8_t unused_was_typeM408 = 3;
	static constexpr uint8_t typeBootloaderName = 4;
	static constexpr uint8_t typeBoardUniqueId = 5;
	static constexpr uint8_t typeDiagnosticsPart0 = 100;
	// Other parts of the diagnostics reply use 101, 102 etc. so keep these free

	uint16_t requestId : 12,
			 param : 4;								// M122 P parameter
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

struct __attribute__((packed)) CanMessageSetHeaterTemperatureV1
{
	static constexpr CanMessageType messageType = CanMessageType::setHeaterTemperatureV1;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heaterNumber : 8,
			 zero2 : 5,
			 function : 3;
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

struct __attribute__((packed)) CanMessageHeaterModelV3
{
	static constexpr CanMessageType messageType = CanMessageType::heaterModelV3;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heater : 8,
			 enabled : 1,
			 inverted : 1,
			 _obsolete_was_pidParametersOverridden : 1,	// this is now unused because we no longer support overriding PID parameters
			 zero2 : 5;
	HeaterModel basicModel;
	float maxPwm;

	// The next 3 are used only if pidParametersOverridden is true
	float _obsolete_was_kP;								// controller (not model) gain
	float _obsolete_was_recipTi;						// reciprocal of controller integral time
	float _obsolete_was_tD;								// controller differential time

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; _obsolete_was_pidParametersOverridden = 0; zero2 = 0; }
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

// Request to create an input monitor
struct __attribute__((packed)) CanMessageCreateInputMonitorV1
{
	static constexpr CanMessageType messageType = CanMessageType::createInputMonitorV1;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	int32_t threshold;			// analog threshold, or zero if digital. Negative means the reading falls to the threshold on trigger instead of rising to it
	uint16_t minInterval;
	char pinName[54];			// null terminated

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
	size_t GetActualDataLength() const noexcept { return 2 * sizeof(uint16_t) + sizeof(uint32_t) + sizeof(RemoteInputHandle) + Strnlen(pinName, sizeof(pinName)/sizeof(pinName[0])); }
	size_t GetMaxPinNameLength(size_t dataLength) const noexcept { return dataLength - (2 * sizeof(uint16_t) + sizeof(uint32_t) + sizeof(RemoteInputHandle)); }
};

// Request to reconfigure an input monitor
struct __attribute__((packed)) CanMessageChangeInputMonitorV1
{
	static constexpr CanMessageType messageType = CanMessageType::changeInputMonitorV1;

	uint16_t requestId : 12,
			 zero : 4;
	RemoteInputHandle handle;
	uint32_t param;
	uint8_t action;

	static constexpr uint8_t actionDontMonitor = 0,					// stop sending status change messages
							actionDoMonitor = 1,					// send status change messages
							actionDelete = 2,						// delete this handle
							actionChangeThreshold = 3,				// change the threshold to param (a signed value, see CanMessageCreateInputMonitorV1) and set standard mode
							actionChangeMinInterval = 4,			// change the minimum interval to param and set standard mode
							actionReturnPinName = 5,				// return the pin name
							actionSetDriveLevel = 6,				// set the drive level to param, only for scanning Z probes
							actionSelectTouchMode = 7,				// select touch mode and set sensitivity to param, only for scanning Z probes
							actionTare = 8;							// tare an analog input in the mode given by param, the baseline is returned as a standard reply data word

	// When the action is actionTare, param selects the tare mode
	static constexpr uint32_t paramTareAndHold = 0,					// latch the baseline and hold it until the next tare, used while a probing move is in progress
							  paramTareAndTrack = 1,				// latch the baseline and let it track slow drift afterwards
							  paramTrackOnly = 2;					// resume tracking from the held baseline without latching, used when a probing move ends with the nozzle possibly still loaded

	// When the action is actionSetDriveLevel, some values of param define a special action:
	static constexpr uint32_t paramAutoCalibrateDriveLevelAndReport = 0xFFFFFFFFu, paramReportDriveLevel = 0xFFFFFFFEu;
	static constexpr uint32_t paramDriveLevelMask = 0x1F;			// bottom 5 bits are the drive level
	static constexpr unsigned int paramOffsetShift = 5;				// remaining bits are the offset
	static constexpr uint32_t maxParamOffset = ((uint32_t)1 << (32 - paramOffsetShift)) - 1;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Struct to represent an analog handle and the reading from it
// These are allocated in an array starting on a 2-or 4-byte boundary. So field 'handle' is correctly aligned but 'reading' isn't.
struct __attribute__((packed)) AnalogHandleDataV0
{
	RemoteInputHandle handle;
	int32_t reading;						// note, this will not be aligned!
};

// Struct to represent an analog handle and the reading from it
// These are allocated in an array starting on 4-byte boundary. So all fields will be aligned correctly.
struct __attribute__((packed)) AnalogHandleDataV1
{
	RemoteInputHandle handle;
	uint16_t when;							// lower 16 bits of the system step tick count when the new value was recorded
	int32_t reading;						// the handle value
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
// If just the 'on' bit is set, we are asking for heater tuning to start.
// If 'on' and 'calibrate' are both set, we are asking for calibration to start.
// If just 'calibrate' is set, we are asking whether calibration has completed.
struct __attribute__((packed)) CanMessageHeaterTuningCommand
{
	static constexpr CanMessageType messageType = CanMessageType::heaterTuningCommand;

	uint16_t requestId : 12,
			 zero : 4;
	uint32_t heaterNumber : 8,
			 on : 1,
			 calibrate : 1,						// added for 3.7.0-beta.2
			 zero2 : 22;
	float pwm;
	float lowTemp;
	float highTemp;
	float peakTempDrop;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Set heater feedforward. The receiving board does not reply to this message.
struct __attribute__((packed)) CanMessageHeaterFeedForwardV1
{
	static constexpr CanMessageType messageType = CanMessageType::heaterFeedForwardV1;

	uint16_t zero;
	uint16_t heaterNumber : 8,
			 zero2 : 8;
	float fanPwmFraction;
	float extrusionPwmBoost;
	float extrusionTemperatureBoost;

	void ClearReservedFields() noexcept { zero = 0; zero2 = 0; }
};

// Configure input shaping
struct __attribute__((packed)) CanMessageSetInputShapingV1
{
	static constexpr CanMessageType messageType = CanMessageType::setInputShapingV1;

	struct ShapingPair { float coefficient; uint32_t impulseDelay; };

	uint16_t requestId : 12,
			 zero : 4;

	uint16_t numImpulses;								// the total number of impulses
	ShapingPair impulses[7];							// the coefficients and durations of the impulses

	size_t GetActualDataLength() const noexcept { return (2 * sizeof(uint16_t)) + (numImpulses * sizeof(ShapingPair)); }
	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Enable a stall endstop, or clear all stall endstops
struct __attribute__((packed)) CanMessageEnableStallEndstop
{
	static constexpr CanMessageType messageType = CanMessageType::enableStallEndstop;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t driverNumber;								// the number of the driver we want to enable a stall endstop for
	float speed;										// the speed we will use for the homing move, not relevant if driverNumber == disableAll

	static constexpr uint16_t disableAll = 0xFFFF;		// if driverNumber is this then we disable all stall endstops on this board

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }
};

// Request to set and return the default model for a heater
struct __attribute__((packed)) CanMessageSetDefaultHeaterModel
{
	static constexpr CanMessageType messageType = CanMessageType::setDefaultHeaterModel;

	uint16_t requestId : 12,
			 zero : 4;
	uint16_t heater: 6,
			 heaterFunction : 3,
			 zero2 : 8;

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; zero2 = 0; }
};

// Reply to SetDefaultHeaterModel
struct __attribute__((packed)) CanMessageHeaterModelReport
{
	static constexpr CanMessageType messageType = CanMessageType::heaterModelReport;

	uint32_t requestId : 12,				// the request ID of the message we are replying to - must be in the same place as in a StandardReply
			 resultCode : 4,				// normally a GCodeResult - must be in the same place as in a StandardReply
			 heaterNumber : 6,				// number of the heater reported
			 zero : 10;						// spare
	HeaterModel model;						// the returned model

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
	void ClearReservedFields() noexcept { }
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
	void ClearReservedFields() noexcept { zero = 0; }
};

// This is the standard reply used by many calls. It carries a GCodeResult, some text, and in some cases 8 bits and/or up to three 32-bit words of additional information.
// It can be split into multiple fragments so that the text is not constrained to 60 characters. The data words are carried in fragment 0 only, ahead of the text.
// The layout of requestId and resultCode are common to more than one reply type
struct __attribute__((packed)) CanMessageStandardReply
{
	static constexpr CanMessageType messageType = CanMessageType::standardReply;
	static constexpr size_t MaxNumWords = 3;

	uint32_t requestId : 12,				// the request ID of the message we are replying to
			 resultCode : 4,				// normally a GCodeResult
			 fragmentNumber : 5,			// the fragment number of this message
			 numWords : 2,					// number of 32-bit data words preceding the text, fragment 0 only
			 moreFollows : 1,				// set if this is not the last fragment of the reply
			 extra : 8;						// normally unused, but occasionally carries extra data
	char text[60];							// numWords data words followed by the text

	size_t GetMaxTextLength() const noexcept { return sizeof(text) - numWords * sizeof(uint32_t); }
	char *GetText() noexcept { return text + numWords * sizeof(uint32_t); }
	const char *GetText() const noexcept { return text + numWords * sizeof(uint32_t); }

	// Packed struct, so copy the word out rather than cast to uint32_t*
	uint32_t GetWord(size_t index) const noexcept
	{
		uint32_t word;
		memcpy(&word, text + index * sizeof(uint32_t), sizeof(word));
		return word;
	}

	void SetWords(const uint32_t *words, size_t count) noexcept
	{
		numWords = count;
		memcpy(text, words, count * sizeof(uint32_t));
	}

	size_t GetTextLength(size_t dataLength) const noexcept
	{
		// can't use min<> here because it hasn't been moved to RRFLibraries yet
		const size_t headerLength = (numWords + 1) * sizeof(uint32_t);
		return (dataLength <= headerLength) ? 0 : Strnlen(GetText(), (dataLength < headerLength + GetMaxTextLength()) ? dataLength - headerLength : GetMaxTextLength());
	}

	size_t GetActualDataLength(size_t textLength) const noexcept
	{
		return textLength + (numWords + 1) * sizeof(uint32_t);
	}

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; fragmentNumber = 0; numWords = 0; moreFollows = 0; extra = 0; }
};

// Response to the ReadInputsRequest. The requestID and resultCode must be in the same place as for a standard reply.
struct __attribute__((packed)) CanMessageReadInputsReplyV0
{
	static constexpr CanMessageType messageType = CanMessageType::readInputsReplyV0;

	uint32_t requestId : 12,				// the request ID of the message we are replying to - must be in the same place as in a StandardReply
			 resultCode : 4,				// normally a GCodeResult - must be in the same place as in a StandardReply
			 numReported : 4,				// number of input handles reported
			 zero : 12;						// spare
	AnalogHandleDataV0 results[10];

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }

	size_t GetActualDataLength() noexcept
	{
		return sizeof(uint32_t) + numReported * sizeof(results[0]);
	}
};

// Response to the ReadInputsRequest. The requestID and resultCode must be in the same place as for a standard reply.
struct __attribute__((packed)) CanMessageReadInputsReplyV1
{
	static constexpr CanMessageType messageType = CanMessageType::readInputsReplyV1;

	uint32_t requestId : 12,				// the request ID of the message we are replying to - must be in the same place as in a StandardReply
			 resultCode : 4,				// normally a GCodeResult - must be in the same place as in a StandardReply
			 numReported : 4,				// number of input handles reported
			 zero : 12;						// spare
	AnalogHandleDataV1 results[7];

	void SetRequestId(CanRequestId rid) noexcept { requestId = rid; zero = 0; }

	size_t GetActualDataLength() noexcept
	{
		return sizeof(uint32_t) + numReported * sizeof(results[0]);
	}
};

struct ParamDescriptor;

// Generic message. These are always used in conjunction with a ParamTable that is know to both sender and receiver.
// The table lists the parameters, each one defined by the parameter letter and the type of parameter.
// The paramMap bitmap indicates which parameters are present in the data. They are provided in the same order as in the ParamTable.
struct __attribute__((packed)) CanMessageGeneric
{
	uint32_t requestId : 12,
			 paramMap : 20;
	uint8_t data[60];

	void DebugPrint(const ParamDescriptor *_ecv_array _ecv_null pt = nullptr) const noexcept;

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

	void ClearReservedFields() noexcept { }
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

	void ClearReservedFields() noexcept { }
};

// Message used by expansion boards running firmware 3.4.0beta4 and earlier to announce their presence on the CAN bus to other boards
struct __attribute__((packed)) CanMessageAnnounceV0
{
	static constexpr CanMessageType messageType = CanMessageType::announceV0;

	uint32_t timeSinceStarted;				// how long since we started up
	uint32_t numDrivers: 8,					// the number of motor drivers on this board
			 zero : 24;						// for future expansion, set to zero
	char boardTypeAndFirmwareVersion[56];	// the type short name of this board followed by '|' and the firmware version

	void ClearReservedFields() noexcept { zero = 0; }

	size_t GetActualDataLength() const noexcept
			{ return (2 * sizeof(uint32_t)) + Strnlen(boardTypeAndFirmwareVersion, sizeof(boardTypeAndFirmwareVersion)/sizeof(boardTypeAndFirmwareVersion[0])); }

	static size_t GetMaxTextLength(size_t dataLength) noexcept { return dataLength - (2 * sizeof(uint32_t)); }
};

// Message used by expansion boards running firmware 3.4.0beta5 and later to announce their presence on the CAN bus to other boards
struct __attribute__((packed)) CanMessageAnnounceV1
{
	static constexpr CanMessageType messageType = CanMessageType::announceV1;

	uint32_t timeSinceStarted;				// how long since we started up
	uint8_t uniqueId[16];					// the unique ID of this board
	uint8_t numDrivers: 4,					// the number of motor drivers on this board
			usesUf2Binary : 1,				// set if this board takes a main firmware binary in .uf2 format
			isReconnect : 1,				// set if this board didn't reset but is re-announcing after losing and regaining time sync
			wasShutDown : 1,				// set if this board switched its heaters off because time sync was lost for longer than the connection timeout
			zero : 1;						// for future expansion, set to zero
	char boardTypeAndFirmwareVersion[43];	// the type short name of this board followed by '|' and the firmware version

	size_t GetActualDataLength() const noexcept
			{ return sizeof(timeSinceStarted) + sizeof(uniqueId) + sizeof(uint8_t) + Strnlen(boardTypeAndFirmwareVersion, sizeof(boardTypeAndFirmwareVersion)/sizeof(boardTypeAndFirmwareVersion[0])); }

	static size_t GetMaxTextLength(size_t dataLength) noexcept { return dataLength - (sizeof(timeSinceStarted) + sizeof(uniqueId) + sizeof(uint8_t)); }

	void ClearReservedFields() noexcept { zero = 0; }
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

	void ClearReservedFields() noexcept { }
};

// Message sent by an expansion board when one of its monitored inputs has changed state
struct __attribute__((packed)) CanMessageInputChangedV1
{
	static constexpr CanMessageType messageType = CanMessageType::inputStateChangedV1;

	uint16_t states;						// 1 bit per reported handle
	uint8_t numHandles;
	uint8_t zero;
	AnalogHandleDataV0 results[10];

	// Add an entry. 'states' and 'numHandles' must be cleared to zero before adding the first one. Return true if successful, false if message is full.
	bool AddEntry(uint16_t h, int32_t val, bool state) noexcept
	{
		if (numHandles < sizeof(results)/sizeof(results[0]))
		{
			if (state)
			{
				states |= 1ul << numHandles;
			}
			results[numHandles].handle.Set(h);
			StoreLEI32(&results[numHandles].reading, val);
			++numHandles;
			return true;
		}
		return false;
	}

	// Get the handle from one of the result values. 'results' is 4-byte allocated and each entry is 6 bytes long, so the 2-byte handle is always 2-byte aligned.
	RemoteInputHandle GetEntryHandle(size_t index) const noexcept { return results[index].handle; }

	// Get the reading from one of the result values. 'results' is 4-byte allocated and each entry is 6 bytes long, so the 4-byte handle is not always 4-byte aligned.
	int32_t GetEntryReading(size_t index) const noexcept { return LoadLEI32(&results[index].reading); }

	size_t GetActualDataLength() const noexcept
	{
		return sizeof(states) + sizeof(numHandles) + sizeof(zero) + (numHandles * sizeof(results[0]));
	}

	void ClearReservedFields() noexcept { zero = 0; }
};

// Message sent by an expansion board when one of its monitored inputs has changed state
struct __attribute__((packed)) CanMessageInputChangedV2
{
	static constexpr CanMessageType messageType = CanMessageType::inputStateChangedV2;

	uint16_t states;						// 1 bit per reported handle
	uint8_t numHandles;
	uint8_t zero;
	AnalogHandleDataV1 results[7];			// this is on a 4-byte boundary

	// Add an entry. 'states' and 'numHandles' must be cleared to zero before adding the first one. Return true if successful, false if message is full.
	bool AddEntry(uint16_t h, uint32_t whenStateChanged, int32_t val, bool state) noexcept
	{
		if (numHandles < sizeof(results)/sizeof(results[0]))
		{
			if (state)
			{
				states |= 1ul << numHandles;
			}
			results[numHandles].handle.Set(h);
			results[numHandles].reading = val;
			results[numHandles].when = (uint16_t)whenStateChanged;
			++numHandles;
			return true;
		}
		return false;
	}

	// Get the handle from one of the result values
	RemoteInputHandle GetEntryHandle(size_t index) const noexcept { return results[index].handle; }

	// Get the reading from one of the result values
	int32_t GetEntryReading(size_t index) const noexcept { return results[index].reading; }

	// Get the change time stamp for one of the result values
	uint16_t GetWhen(size_t index) const noexcept { return results[index].when; }

	size_t GetActualDataLength() const noexcept
	{
		return sizeof(states) + sizeof(numHandles) + sizeof(zero) + (numHandles * sizeof(results[0]));
	}

	void ClearReservedFields() noexcept { zero = 0; }
};

// Message sent by expansion boards to report their general health
struct __attribute__((packed)) CanMessageBoardStatusV0
{
	static constexpr CanMessageType messageType = CanMessageType::boardStatusReportV0;

	uint32_t hasVin : 1,
			 hasV12 : 1,
			 hasMcuTemp : 1,
			 hasAccelerometer : 1,
			 hasClosedLoop : 1,
			 hasInductiveSensor : 1,
			 zero : 10,							// reserved for future use
			 hasMovementDelay : 1,
			 numAnalogHandles : 3,				// how many instances of AnalogHandleData we append
			 zero2 : 12;
	union
	{
		int32_t neverUsedRam;					// this field present if hasMovementDelay is false
		uint32_t movementDelay;					// this field present if hasMovementDelay is true
	};
	MinCurMax values[3];						// values of none, some or all of Vin, V12 and CPU temperature
	// After the last present MinCurMax value the data for some analog handles follows (max 4 if all of Vin/V12/mcuTemp are supported)

	void Clear() noexcept
	{
		hasVin = hasV12 = hasMcuTemp = hasMovementDelay = hasAccelerometer = hasClosedLoop = hasInductiveSensor = false;
		numAnalogHandles = 0;
	}

	size_t GetAnalogHandlesOffset() const noexcept
	{
		const unsigned int numMinCurMaxValues = hasVin + hasV12 + hasMcuTemp;
		return 2 * sizeof(uint32_t) + numMinCurMaxValues * sizeof(values[0]);
	}

	size_t GetMaxAnalogHandleSpace() const noexcept
	{
		return 64 - GetAnalogHandlesOffset();
	}

	size_t GetActualDataLength() const noexcept
	{
		return GetAnalogHandlesOffset() + numAnalogHandles * sizeof(AnalogHandleDataV0);
	}

	void ClearReservedFields() noexcept { zero = 0; zero2 = 0; }
};

// Message sent by expansion boards to report their general health
struct __attribute__((packed)) CanMessageBoardStatusV1
{
	static constexpr CanMessageType messageType = CanMessageType::boardStatusReportV1;

	uint32_t hasVin : 1,
			 hasV12 : 1,
			 hasMcuTemp : 1,
			 hasAccelerometer : 1,
			 hasClosedLoop : 1,
			 hasInductiveSensor : 1,
			 zero : 10,							// reserved for future use
			 hasMovementDelay : 1,
			 numAnalogHandles : 3,				// how many instances of AnalogHandleData we append
			 zero2 : 12;
	union
	{
		int32_t neverUsedRam;					// this field present if hasMovementDelay is false
		uint32_t movementDelay;					// this field present if hasMovementDelay is true
	};
	ShortMinCurMax shortValues[3];				// values of none, some or all of Vin, V12 and CPU temperature
	// After the last present ShortMinCurMax value the data for some analog handles follows (max 5 if all of Vin/V12/mcuTemp are supported)

	void Clear() noexcept
	{
		hasVin = hasV12 = hasMcuTemp = hasMovementDelay = hasAccelerometer = hasClosedLoop = hasInductiveSensor = false;
		numAnalogHandles = 0;
	}

	size_t GetAnalogHandlesOffset() const noexcept
	{
		const unsigned int numMinCurMaxValues = hasVin + hasV12 + hasMcuTemp;
		return 2 * sizeof(uint32_t) + numMinCurMaxValues * sizeof(shortValues[0]);
	}

	size_t GetMaxAnalogHandleSpace() const noexcept
	{
		return 64 - GetAnalogHandlesOffset();
	}

	size_t GetActualDataLength() const noexcept
	{
		return GetAnalogHandlesOffset() + numAnalogHandles * sizeof(AnalogHandleDataV1);
	}

	void ClearReservedFields() noexcept { zero = 0; zero2 = 0; }
};

// Struct to represent driver status. If this is changed then CanMessageDriversStatus must be replaced by a new version.
struct __attribute__((packed)) OpenLoopStatus
{
	uint32_t status;
};

// Struct to represent driver status including closed loop data. If this is changed then CanMessageDriversStatus must be replaced by a new version.
struct __attribute__((packed)) ClosedLoopStatus
{
	uint32_t status;
	float16_t averageCurrentFraction;
	float16_t maxCurrentFraction;
	float16_t rmsPositionError;
	float16_t maxAbsPositionError;
};

// Message sent by expansion boards to report the status of their drivers
struct __attribute__((packed)) CanMessageDriversStatus
{
	static constexpr CanMessageType messageType = CanMessageType::driversStatusReport;

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
	}

	void ClearReservedFields() noexcept { zero = 0; zero2 = 0; }
};

// This has to be declared outside struct CanMessageFilamentMonitorsStatusV2 to avoid having to include this file in FilamentMonitor.h
struct __attribute__((packed)) FilamentMonitorDataV2
{
	uint32_t position : 12,				// raw position from the sensor
			 filamentPresentValid : 1,	// true if the filamentPresent bit is meaningful
			 filamentPresent : 1,		// true if the sensor reports filament present, only valid if filamentPresentValid is set
			 motionDetected : 1,		// true if filament movement was detected within the last FilamentMonitorMotionLatchTime
			 extraDataValid : 1,		// true if the extraData field is meaningful
			 extraData : 8,				// AGC of a rotating magnet monitor or shutter of a laser monitor, only valid if extraDataValid is set
			 status : 4,				// standard filament status
			 zero2 : 2,					// reserved for future use
			 hasLiveData : 1;			// true if the following fields are meaningful for this sensor

	int32_t minPercentage : 10,
			maxPercentage : 10,
			avgPercentage : 10,
			lastPercentage : 10,		// declaring this struct with attribute packed allows this to straddle word boundaries
			calibrationLength : 24;

	void ClearReservedFields() noexcept { extraDataValid = extraData = 0; zero2 = 0; }
};

// Message sent by expansion boards to report the status of their filament monitors
struct __attribute__((packed)) CanMessageFilamentMonitorsStatusV2
{
	static constexpr CanMessageType messageType = CanMessageType::filamentMonitorsStatusReportV2;

	uint32_t driversReported : 8,			// bitmap of driver numbers with associated filament monitors reported in this message
			 zero : 24;
	FilamentMonitorDataV2 data[5];

	size_t GetActualDataLength() const noexcept
	{
		return sizeof(uint32_t) + (Bitmap<uint32_t>(driversReported).CountSetBits() * sizeof(data[0]));
	}

	void SetStandardFields(Bitmap<uint32_t> drivers) noexcept
	{
		driversReported = drivers.GetRaw();
	}

	void ClearReservedFields() noexcept { zero = 0; }
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
	}

	void ClearReservedFields() noexcept { zero = 0; }
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

	void ClearReservedFields() noexcept { zero = 0; }
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

	void ClearReservedFields() noexcept { zero = 0; zero2 = 0; }
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

	void ClearReservedFields() noexcept { zero = 0; }
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

	void ClearReservedFields() noexcept { }
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
	CanMessageMovementLinearShaped moveLinearShaped;
	CanMessageReturnInfo getInfo;
	CanMessageSetHeaterTemperatureV1 setTemp;
	CanMessageStandardReply standardReply;
	CanMessageFirmwareUpdateRequest firmwareUpdateRequest;
	CanMessageFirmwareUpdateResponse firmwareUpdateResponse;
	CanMessageSensorTemperatures sensorTemperaturesBroadcast;
	CanMessageHeatersStatus heatersStatusBroadcast;
	CanMessageHeaterModelV3 heaterModelV3;
	CanMessageMultipleDrivesRequest<uint16_t> multipleDrivesRequestUint16;
	CanMessageMultipleDrivesRequest<float> multipleDrivesRequestFloat;
	CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping> multipleDrivesStepsPerUnitAndMicrostepping;
	CanMessageMultipleDrivesRequest<DriverStateControl> multipleDrivesRequestDriverState;
	CanMessageMultipleDrivesRequest<ShortPressureAdvanceParameters> multipleDrivesRequestPressureAdvance;
	CanMessageUpdateYourFirmware updateYourFirmware;
	CanMessageFanParameters fanParameters;
	CanMessageSetFanSpeed setFanSpeed;
	CanMessageSetHeaterFaultDetectionParameters setHeaterFaultDetection;
	CanMessageSetHeaterMonitors setHeaterMonitors;
	CanMessageSetInputShapingV1 setInputShapingV1;
	CanMessageCreateInputMonitorV1 createInputMonitorV1;
	CanMessageChangeInputMonitorV1 changeInputMonitorV1;
	CanMessageInputChangedV1 inputChangedV1;
	CanMessageInputChangedV2 inputChangedV2;
	CanMessageFansReport fansReport;
	CanMessageWriteGpio writeGpio;
	CanMessageSetAddressAndNormalTiming setAddressAndNormalTiming;
	CanMessageAnnounceV0 announceV0;
	CanMessageAnnounceV1 announceV1;
	CanMessageAcknowledgeAnnounce acknowledgeAnnounce;
	CanMessageDiagnosticTest diagnosticTest;
	CanMessageReadInputsRequest readInputsRequest;
	CanMessageReadInputsReplyV0 readInputsReplyV0;
	CanMessageReadInputsReplyV1 readInputsReplyV1;
	CanMessageBoardStatusV0 boardStatusV0;
	CanMessageBoardStatusV1 boardStatusV1;
	CanMessageDriversStatus driversStatus;
	CanMessageFilamentMonitorsStatusV2 filamentMonitorsStatusV2;
	CanMessageCreateFilamentMonitor createFilamentMonitor;
	CanMessageDeleteFilamentMonitor deleteFilamentMonitor;
	CanMessageHeaterTuningCommand heaterTuningCommand;
	CanMessageHeaterTuningReport heaterTuningReport;
	CanMessageHeaterFeedForwardV1 heaterFeedForwardV1;
	CanMessageStartAccelerometer startAccelerometer;
	CanMessageAccelerometerData accelerometerData;
	CanMessageStartClosedLoopDataCollection startClosedLoopDataCollection;
	CanMessageClosedLoopData closedLoopData;
	CanMessageEvent event;
	CanMessageDebugText debugText;
	CanMessageEnableStallEndstop enableStallEndstop;
	CanMessageSetDefaultHeaterModel setDefaultHeaterModel;
	CanMessageHeaterModelReport heaterModelReport;
};

static_assert(sizeof(CanMessage) <= 64, "CAN message too big");		// check none of the messages is too large

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
