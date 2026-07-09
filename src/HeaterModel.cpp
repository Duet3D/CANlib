/*
 * HeaterModel.cpp
 *
 *  Created on: 29 Apr 2026
 *      Author: David
 */

#include "HeaterModel.h"
#include <General/SimpleMath.h>
#include <cmath>

#define SQRT_FAN_SCALING		(0)				// Prusa apparently thinks that cooling goes as the square root of fan speed, but I can't find any evidence of thia

// Calculate the cooling rate excluding the fan contribution
float HeaterModel::GetBasicCoolingRate(float temperatureRise) const noexcept
{
	temperatureRise *= 0.01;

	// If the temperature rise is negative then we must not try to raise it to a non-integral power!
	const float adjustedTemperatureRise = (temperatureRise < 0.0) ? -powf(-temperatureRise, coolingRateExponent) : powf(temperatureRise, coolingRateExponent);
	return basicCoolingRate * adjustedTemperatureRise;
}

// Calculate the extra cooling rate provided by the part cooling fan
float HeaterModel::GetFanCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
#if SQRT_FAN_SCALING
	return temperatureRise * 0.01 * fanCoolingRate * fastSqrtf(fanPwm);
#else
	return temperatureRise * 0.01 * fanCoolingRate * fanPwm;
#endif
}

// Calculate the total cooling rate, excluding cooling caused by the filament
float HeaterModel::GetTotalCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	return GetBasicCoolingRate(temperatureRise) + GetFanCoolingRate(temperatureRise, fanPwm);
}

// Calculate the expected heating rate
float HeaterModel::GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept
{
	const float virtualPwm = (standardVoltage < 10.0 || actualVoltage < 10.0) ? heaterPwm : heaterPwm * fsquare(actualVoltage/standardVoltage);
	return heatingRate * (virtualPwm - filamentPwm) - GetTotalCoolingRate(temperatureRise, fanPwm);
}

// Calculate the expected PWM needed to maintain the specified temperature rise above ambient.
// Caution: result may be < 0.0 if the temperature rise is negative, or > 1.0 if we can't maintain the temperature
float HeaterModel::GetExpectedPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept
{
	const float coolingRate = GetTotalCoolingRate(temperatureRise, fanPwm);
	const float virtualPwm = coolingRate/heatingRate + filamentPwm;
	return (standardVoltage < 10.0 || actualVoltage < 10.0) ? virtualPwm : virtualPwm * fsquare(standardVoltage/actualVoltage);
}

// Calculate the change in required heater PWM due to a change in fan PWM
float HeaterModel::GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept
{
#if SQRT_FAN_SCALING
	return temperatureRise * 0.01 * fanCoolingRate * (fastSqrtf(newFanPwm) - fastSqrtf(oldFanPwm)) / heatingRate;
#else
	return temperatureRise * 0.01 * fanCoolingRate * (newFanPwm - oldFanPwm) / heatingRate;
#endif
}

// Estimate the maximum temperature rise that this heater gives at full power
float HeaterModel::EstimateMaxTemperatureRise() const noexcept
{
	return 100.0 * powf(heatingRate/basicCoolingRate, 1.0/coolingRateExponent);
}

// Calculate the basic cooling rate from measurements. This is the inverse of GetBasicCoolingRate but we can assume that temperatureRise is positive.
float HeaterModel::CalculateBasicCoolingRate(float temperatureRise, float coolingRate) const noexcept
{
	const float adjustedTemperatureRise = powf(temperatureRise * 0.01, coolingRateExponent);
	return coolingRate/adjustedTemperatureRise;
}

// Calculate the fan cooling rate from measurements. This is the inverse of GetFanCoolingRate.
float HeaterModel::CalculateFanCoolingRate(float temperatureRise, float coolingRate, float fanPwm) const noexcept
{
#if SQRT_FAN_SCALING
	return (coolingRate * 100.0)/(temperatureRise * fastSqrtf(fanPwm));
#else
	return (coolingRate * 100.0)/(temperatureRise * fanPwm);
#endif
}

// End
