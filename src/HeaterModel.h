/*
 * HeaterModel.h
 *
 *  Created on: 6 Feb 2026
 *      Author: David
 */

#ifndef SRC_HEATERMODEL_H_
#define SRC_HEATERMODEL_H_

#include <cstdint>

// This struct defines the heater model used by RRF.
// CAUTION! This struct is sent as-is in CanMessageHeaterModelReport, therefore if it is changed then that message must be replaced.
struct HeaterModel
{
	float heatingRate;						// the rate at which the heater heats up at 25C and full PWM with no cooling
	float basicCoolingRate;					// the rate at which the heater cools down when it is 100C above ambient and the print cooling fan is off
	float fanCoolingRate;					// the additional cooling rate at 100C above ambient with the print cooling fan on at full PWM
	float coolingRateExponent;				// how the basic cooling rate varies with temperature difference
	float deadTime;							// how long between the heating power changing and the temperature sensor noticing it
	float temperatureCoefficient;			// how much the heating power increases per degC above 25C. Negative for PTC heaters.
	float typicalTemperature;				// temperature at which the PID parameters are calculated
	float standardVoltage;					// power voltage reading at which tuning was done, or 0 if unknown. Heater power is assumed to vary as the square of voltage.
	float fzero;							// reserved for expansion, e.g. cooling rate due to extrusion
	uint32_t usePid : 1,
			 zero : 31;

	float GetBasicCoolingRate(float temperatureRise) const noexcept;										// Calculate the cooling rate excluding the fan contribution
	float GetFanCoolingRate(float temperatureRise, float fanPwm) const noexcept;							// Calculate the extra cooling rate provided by the part cooling fan
	float GetTotalCoolingRate(float temperatureRise, float fanPwm) const noexcept;							// Calculate the total cooling rate, excluding cooling caused by the filament
	float GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept;		// Calculate the expected heating rate
	float GetExpectedPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept;	// Calculate the required PWM to maintain the temperature rise
	float GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept;	// Calculate the change in heating power required to compensate for a change in fan speed
	float EstimateMaxTemperatureRise() const noexcept;														// Estimate the maximum temperature rise that this heater will produce at full power
	float CalculateBasicCoolingRate(float temperatureRise, float coolingRate) const noexcept;				// Calculate the basic cooling rate from measurements
	float CalculateFanCoolingRate(float temperatureRise, float coolingRate, float fanPwm) const noexcept;	// Calculate the fan cooling rate from measurements
};

// These parameters are about right for an E3Dv6 hot end with 30W heater, cooling time constant is about 140 seconds with the fan off
constexpr HeaterModel DefaultToolHeaterModel =
{
	.heatingRate = 2.43,
	.basicCoolingRate = 0.56,
	.fanCoolingRate = 0.0,
	.coolingRateExponent = 1.35,
	.deadTime = 5.5,
	.temperatureCoefficient = 0.0,
	.typicalTemperature = 220.0,
	.standardVoltage = 0.0,
	.fzero = 0.0,
	.usePid = true,
	.zero = 0
};

// These parameters are about right for a typical PCB bed heater that maxes out at 110C and has a cooling time constant of 700 seconds
constexpr HeaterModel DefaultBedHeaterModel =
{
	.heatingRate = 0.13,
	.basicCoolingRate = 0.15,
	.fanCoolingRate = 0.0,
	.coolingRateExponent = 1.35,
	.deadTime = 10.0,
	.temperatureCoefficient = 0.0,
	.typicalTemperature = 60.0,
	.standardVoltage = 0.0,
	.fzero = 0.0,
	.usePid = false,
	.zero = 0
};

// These parameters are copied from the bed heater parameters, except that the dead time is increased
constexpr HeaterModel DefaultChamberHeaterModel =
{
	.heatingRate = 0.13,
	.basicCoolingRate = 0.15,
	.fanCoolingRate = 0.0,
	.coolingRateExponent = 1.35,
	.deadTime = 30.0,
	.temperatureCoefficient = 0.0,
	.typicalTemperature = 60.0,
	.standardVoltage = 0.0,
	.fzero = 0.0,
	.usePid = false,
	.zero = 0
};

#endif /* SRC_HEATERMODEL_H_ */
