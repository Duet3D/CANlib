/*
 * HeaterModel.cpp
 *
 *  Created on: 29 Apr 2026
 *      Author: David
 */

#include "HeaterModel.h"
#include <General/SimpleMath.h>
#include <cmath>

#define SQRT_FAN_SCALING		(1)

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
	temperatureRise *= 0.01;
	// If the temperature rise is negative then we must not try to raise it to a non-integral power!
	const float adjustedTemperatureRise = (temperatureRise < 0.0) ? -powf(-temperatureRise, coolingRateExponent) : powf(temperatureRise, coolingRateExponent);
	return basicCoolingRate * adjustedTemperatureRise + GetFanCoolingRate(temperatureRise, fanPwm);
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

// End
