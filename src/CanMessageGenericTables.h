/*
 * CanMessageGenericTables.h
 *
 *  Created on: 21 Aug 2021
 *      Author: David
 */

#ifndef SRC_CANMESSAGEGENERICTABLES_H_
#define SRC_CANMESSAGEGENERICTABLES_H_

#include "CanMessageGenericTableFormat.h"

// Parameter tables for various messages that use the generic format.
// For backwards compatibility, any new parameters added to a table must be added at the end of that table.
// Maximum length of data is 60 bytes.

constexpr ParamDescriptor M42Params[] =
{
	UINT16_PARAM('P'),
	FLOAT_PARAM('S'),
	END_PARAMS
};

constexpr ParamDescriptor M150Params[] =
{
	UINT16_PARAM('E'),		// strip number
	UINT16_PARAM('S'),		// number of LEDs to set
	UINT8_PARAM('R'),		// red intensity
	UINT8_PARAM('U'),		// green intensity
	UINT8_PARAM('B'),		// blue intensity
	UINT8_PARAM('W'),		// white intensity
	UINT8_PARAM('P'),		// brightness 0-255
	UINT8_PARAM('Y'),		// alternative brightness 0-31
	UINT8_PARAM('F'),		// 'more follows' flag
	END_PARAMS
};

constexpr ParamDescriptor M280Params[] =
{
	UINT16_PARAM('P'),
	UINT16_PARAM('S'),
	END_PARAMS
};

constexpr ParamDescriptor M308NewParams[] =
{
	FLOAT_PARAM('T'),
	FLOAT_PARAM('B'),
	FLOAT_PARAM('C'),
	FLOAT_PARAM('R'),
	INT16_PARAM('L'),
	INT16_PARAM('H'),
	UINT8_PARAM('F'),
	UINT8_PARAM('S'),
	UINT8_PARAM('W'),
	CHAR_PARAM('K'),
	REDUCED_STRING_PARAM('Y'),
	REDUCED_STRING_PARAM('P'),
	FLOAT16_PARAM('U'),
	FLOAT16_PARAM('V'),
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

// Configure closed loop stepper motor
constexpr ParamDescriptor M569Point1Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('T'),
	FLOAT_ARRAY_PARAM('E', 2),
	FLOAT_PARAM('C'),
	FLOAT_PARAM('R'),
	FLOAT_PARAM('I'),
	FLOAT_PARAM('D'),
	FLOAT_PARAM('H'),					// no longer used, retained for backwards compatibility
	UINT16_PARAM('S'),					// steps/rev added for EXP1HCL firmware 3.5 compatibility
	FLOAT_PARAM('V'),					// velocity feedforward term added in 3.5beta2
	FLOAT_PARAM('A'),					// acceleration feedforward term added in 3.5beta4
	FLOAT_PARAM('Q'),					// torque constant in Nm per A added in 3.5 post beta4
	END_PARAMS
};

// Read or write stepper driver register
constexpr ParamDescriptor M569Point2Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('R'),
	UINT32_PARAM('V'),
	END_PARAMS
};

// Set driver torque mode
constexpr ParamDescriptor M569Point4Params[] =
{
	LOCAL_DRIVER_PARAM('P'),			// driver
	FLOAT_PARAM('T'),					// requested torque in Nm
	FLOAT_PARAM('V'),					// maximum speed to move at
	END_PARAMS
};

// Note: M569Point6Params_StatusOnly must be kept in step with this!
constexpr ParamDescriptor M569Point6Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('V'),
	FLOAT_PARAM('S'),					// speed parameter (in full steps/sec) for step tuning move (can't use V because it is taken already)
	FLOAT_PARAM('A'),					// acceleration parameter (in full steps/sec^2) for step tuning move
	END_PARAMS
};

// This is the same as M569Point6Params except that it doesn't pick up the V parameter from the GCodeBuffer, and we don't need the extra parameters
constexpr ParamDescriptor M569Point6Params_StatusOnly[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('v'),					// changed to lowercase so that we don't pick up this parameter from the GCode command
	END_PARAMS
};

constexpr ParamDescriptor M569Point7Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	REDUCED_STRING_PARAM('C'),
	FLOAT_PARAM('V'),					// brake voltage added at 3.5beta2
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
	REDUCED_STRING_PARAM('C'),
	FLOAT_PARAM('K'),					// tacho pulses/rev added at 3.5
	END_PARAMS
};

constexpr ParamDescriptor M950GpioParams[] =
{
	UINT16_PARAM('P'),
	PWM_FREQ_PARAM('Q'),
	UINT8_PARAM('S'),					// 1 if servo, 0 if GPIO
	REDUCED_STRING_PARAM('C'),
	END_PARAMS
};

constexpr ParamDescriptor M950LedParams[] =
{
	UINT16_PARAM('E'),					// Strip number
	UINT16_PARAM('U'),					// Maximum number of LEDs
	UINT32_PARAM('Q'),					// SPI frequency (DotStar) or cycle frequency (Neopixel)
	UINT8_PARAM('T'),					// Type (0 = DotStar, 1 = RGB Neopixel, 2 = RGBW Neopixel)
	REDUCED_STRING_PARAM('C'),			// Port name
	UINT16_ARRAY_PARAM('L', 4),			// Timing parameters, used by STM implementation only, implies bit-banged if present
	END_PARAMS
};

// This must cover all parameters used by any supported type of filament monitor, except for the type and extruder number
constexpr ParamDescriptor ConfigureFilamentMonitorParams[] =
{
	UINT8_PARAM('d'),					// this is the driver number on the remote board
	UINT8_PARAM('S'),
	UINT8_PARAM('A'),
	FLOAT_PARAM('L'),
	FLOAT_PARAM('E'),
	UINT16_ARRAY_PARAM('R', 2),
	REDUCED_STRING_PARAM('C'),
	END_PARAMS
};

// Accelerometer settings
constexpr ParamDescriptor M955Params[] =
{
	LOCAL_DRIVER_PARAM('P'),			// accelerometer number
	UINT8_PARAM('I'),					// orientation
	UINT8_PARAM('R'),					// resolution (bits)
	UINT16_PARAM('S'),					// sampling rate
	END_PARAMS
};

// M122 P1 parameters
constexpr ParamDescriptor M122P1Params[] =
{
	FLOAT_ARRAY_PARAM('T', 2),			// MCU temperature limits
	FLOAT_ARRAY_PARAM('V', 2),			// VIN voltage limits
	FLOAT_ARRAY_PARAM('W', 2),			// V12 voltage limits
	FLOAT_ARRAY_PARAM('U', 2),			// thermistor temperature reading limits
	FLOAT_ARRAY_PARAM('F', 2),			// inductive sensor frequency limits
	END_PARAMS
};

#endif /* SRC_CANMESSAGEGENERICTABLES_H_ */
