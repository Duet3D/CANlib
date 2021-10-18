/*
 * RRF3Common.h
 *
 *  Created on: 17 Oct 2021
 *      Author: David
 *
 *  This file contains definitions common to all implementations of RRF3 on Duets, including those that don't support CAN
 */

#ifndef SRC_RRF3COMMON_H_
#define SRC_RRF3COMMON_H_

#include <cstdint>
#include <General/NamedEnum.h>

// Constants etc. that are common across Duet main and expansion boards
constexpr float DefaultThermistorR25 = 100000.0;
constexpr float DefaultThermistorBeta = 4725.0;
constexpr float DefaultThermistorC = 7.060e-8;

constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

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
				 closedLoopNotTuned : 1,		// closed loop driver has not been tuned
				 closedLoopTuningError : 1,		// closed loop tuning failed
				 closedLoopIllegalMove : 1,		// move attempted in closed loop mode when driver not tuned
				 zero2 : 2,						// spare, always zero for now
				 sgresultMin : 10,				// minimum stallguard result seen
				 zero3 : 6;						// reserved for future use - don't use the MSB because it will make the value negative in the OM
	};

	static constexpr unsigned int OtBitPos = 0;
	static constexpr unsigned int OtpwBitPos = 1;
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

// Enum to represent an event type. Earlier values in the list have higher priority.
NamedEnum(EventType, uint8_t, mainBoardPowerFail, heaterFault, driverError, filamentError, driverWarning, mcuTemperatureWarning);

#endif /* SRC_RRF3COMMON_H_ */
