/*
 * HeaterModel.cpp
 *
 *  Created on: 29 Apr 2026
 *      Author: David
 */

#include "HeaterModel.h"
#include <General/SimpleMath.h>
#include <cmath>

// A note on how nozzle cooling varies with fan PWM.
// Tests done tuning an INDX with the dual centrifugal fan cooler at fan PWM 0.4, 0.6, 0.8 and 1.0 yielded fan cooling rates of 1.182, 1.181, 1.123, 1.063.
// The code assumes linear cooling rate with fan PWM and divides by PWM to get these figures, so ideally these values would all be the same.
// A power law fit through these points suggests that cooling rate goes as fan PWM to the power 0.89.
// This is sufficiently close to linear for us to tune at 0.7 PWM and assume linear scaling.
// The cooling will be slightly higher than predicted at low PWM and slightly lower at full PWM.

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
	return temperatureRise * 0.01 * fanCoolingRate * fanPwm;
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
	return temperatureRise * 0.01 * fanCoolingRate * (newFanPwm - oldFanPwm) / heatingRate;
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
	return (coolingRate * 100.0)/(temperatureRise * fanPwm);
}

// End
