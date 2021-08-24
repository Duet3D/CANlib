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

constexpr ParamDescriptor M42Params[] =
{
	UINT16_PARAM('P'),
	FLOAT_PARAM('S'),
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
	STRING_PARAM('C'),
	END_PARAMS
};

constexpr ParamDescriptor M950GpioParams[] =
{
	UINT16_PARAM('P'),
	PWM_FREQ_PARAM('Q'),
	UINT8_PARAM('S'),			// 1 if servo, 0 if GPIO
	REDUCED_STRING_PARAM('C'),
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

constexpr ParamDescriptor M569Point1Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('T'),
	FLOAT_ARRAY_PARAM('E', 2),
	FLOAT_PARAM('C'),
	FLOAT_PARAM('R'),
	FLOAT_PARAM('I'),
	FLOAT_PARAM('D'),
	UINT8_PARAM('L'),
	END_PARAMS
};

constexpr ParamDescriptor M569Point2Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('R'),
	UINT32_PARAM('V'),
	END_PARAMS
};

// Note: M569Point6Params_StatusOnly must be kept in step with this!
constexpr ParamDescriptor M569Point6Params[] =
{
	LOCAL_DRIVER_PARAM('P'),
	UINT8_PARAM('V'),
	END_PARAMS
};

// This is the same as M569Point6Params except that it doesn't pick up the V parameter from the GCodeBuffer
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

#endif /* SRC_CANMESSAGEGENERICTABLES_H_ */
