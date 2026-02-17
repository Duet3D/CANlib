/*
 * TemperatureError.h
 *
 *  Created on: 21 Apr 2016
 *      Author: David
 */

#ifndef TEMPERATUREERROR_H_
#define TEMPERATUREERROR_H_

#include <cstdint>
#include <General/NamedEnum.h>

// Result codes returned by temperature sensor drivers
NamedEnum(TemperatureError, uint8_t,
	ok,
	shortCircuit,
	shortToVcc,
	shortToGround,
	openCircuit,
	timeout,
	ioError,
	hardwareError,
	notReady,
	invalidOutputNumber,
	busBusy,
	badResponse,
	unknownPort,
	notInitialised,
	unknownSensor,
	overOrUnderVoltage,
	badVref,
	badVssa,
	readingTooLow,
	readingTooHigh,
	ambientReadingTooLow,			// for composite sensors that need to read ambient temperature to calculate object temperature
	ambientReadingTooHigh,			// for composite sensors that need to read ambient temperature to calculate object temperature
	unknownError
);

#endif /* TEMPERATUREERROR_H_ */
