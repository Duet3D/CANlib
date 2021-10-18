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
#include <General/StringRef.h>
#include <General/SimpleMath.h>

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
		uint32_t otpw : 1,								// over temperature warning
				 ot : 1,								// over temperature error
				 s2ga : 1,								// short to ground phase A
				 s2gb : 1,								// short to ground phase B
				 s2vsa : 1,								// short to VS phase A
				 s2vsb : 1,								// short to VS phase B
				 ola : 1,								// open load phase A
				 olb : 1,								// open load phase B
				 // The remaining bit assignments do not correspond to TMC2209 bit positions
				 standstill : 1,						// standstill indicator
				 stall : 1,								// stall, or closed loop error exceeded
				 notPresent : 1,						// smart driver not present
				 externalDriverError : 1,				// external driver signalled error
				 closedLoopPositionWarning : 1,			// close to stall, or closed loop warning
				 closedLoopPositionNotMaintained : 1,	// failed to achieve position
				 closedLoopNotTuned : 1,				// closed loop driver has not been tuned
				 closedLoopTuningError : 1,				// closed loop tuning failed
				 closedLoopIllegalMove : 1,				// move attempted in closed loop mode when driver not tuned
				 zero : 5,								// reserved for future use - don't use the MSB because it will make the value negative in the OM
				 sgresultMin : 10;						// minimum stallguard result seen
	};

	static constexpr unsigned int OtBitPos = 0;
	static constexpr unsigned int OtpwBitPos = 1;
	static constexpr unsigned int StandstillBitPos = 8;
	static constexpr unsigned int StallBitPos = 10;
	static constexpr unsigned int SgresultBitPos = 16;

	static constexpr uint32_t ErrorMask =   0b10010101000111110;		// bit positions that usually correspond to errors
	static constexpr uint32_t WarningMask = 0b01001000011000001;		// bit positions that correspond to warnings
	static constexpr uint32_t InfoMask =    0b00100010100000000;		// bit positions that correspond to information

	static_assert((ErrorMask & WarningMask) == 0);
	static_assert((ErrorMask & InfoMask) == 0);
	static_assert((InfoMask & WarningMask) == 0);

	void AppendText(const StringRef& str, unsigned int severity) const noexcept;

private:
	// Strings representing the meaning of each bit in DriverStatus
	static constexpr const char *BitMeanings[] =
	{
		"over temperature warning",
		"over temperature shutdown",
		"phase A short to ground",
		"phase B short to ground",
		"phase A short to Vin",
		"phase B short to Vin",
		"phase A may be disconnected",
		"phase B may be disconnected",
		"standstill",
		"stalled",
		"not present",
		"external driver error",
		"position tolerance exceeded",
		"failed to maintain position",
		"not tuned",
		"tuning failed",
		"move attempted when not tuned"
	};

	static_assert((1u << ARRAY_SIZE(BitMeanings)) - 1 == (ErrorMask | WarningMask | InfoMask));
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

NamedEnum(HeaterFaultType, uint8_t, temperature_rising_too_slowly, exceeded_allowed_excursion, failed_to_read_sensor);

// Enum to represent an event type. Earlier values in the list have higher priority.
NamedEnum(EventType, uint8_t, Main_board_power_failure, Heater_fault, Driver_error, Filament_error, Driver_warning, Mcu_temperature_warning);

union EventParameter
{
	uint32_t uVal;
	float fVal;
	StandardDriverStatus driverStatus;
	FilamentSensorStatus::BaseType filamentStatus;
	HeaterFaultType::BaseType heaterFaultStatus;
};

#endif /* SRC_RRF3COMMON_H_ */
