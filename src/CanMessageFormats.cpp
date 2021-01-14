/*
 * CanMessageFormats.cpp
 *
 *  Created on: 15 Jul 2019
 *      Author: David
 */

#include "CanMessageFormats.h"
#include "General/StringFunctions.h"
#include <cinttypes>
#include <cstring>

extern "C" void debugPrintf(const char* fmt, ...) noexcept __attribute__ ((format (printf, 1, 2)));

// Round up a message length to the size that will actually be sent. Used to ensure that we include trailing null terminators.
size_t CanAdjustedLength(size_t rawLength)
{
	return (rawLength <= 8) ? rawLength
			: (rawLength <= 24) ? (rawLength + 3) & ~3
				: (rawLength < 64) ? (rawLength + 15) & ~15
					: 64;
}

#if 0	// this message is no longer used

void CanMessageMovement::DebugPrint() const noexcept
{
	debugPrintf("Can: %08" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %f %f:",
		whenToExecute, accelerationClocks, steadyClocks, decelClocks, (double)initialSpeedFraction, (double)finalSpeedFraction);
	for (size_t i = 0; i < MaxDriversPerCanSlave; ++i)
	{
		debugPrintf(" %" PRIi32, perDrive[i].steps);
	}
	debugPrintf("\n");
}

#endif

void CanMessageMovementLinear::DebugPrint() const noexcept
{
	debugPrintf("Can: %08" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %f %f:",
		whenToExecute, accelerationClocks, steadyClocks, decelClocks, (double)initialSpeedFraction, (double)finalSpeedFraction);
	for (size_t i = 0; i < numDrivers; ++i)
	{
		debugPrintf(" %" PRIi32, perDrive[i].steps);
	}
	debugPrintf("\n");
}

void CanMessageGeneric::DebugPrint(const ParamDescriptor *pt) const noexcept
{
	if (pt == nullptr)
	{
		debugPrintf("CanG PT: %d D: %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32 "\n",
			paramMap, *reinterpret_cast<const uint32_t*>(data + 4), *reinterpret_cast<const uint32_t*>(data + 8), *reinterpret_cast<const uint32_t*>(data + 12),
			*reinterpret_cast<const uint32_t*>(data + 16), *reinterpret_cast<const uint32_t*>(data + 20), *reinterpret_cast<const uint32_t*>(data + 24),
			*reinterpret_cast<const uint32_t*>(data + 28));
	}
	else
	{
		debugPrintf("CanG");
		uint32_t pm = paramMap;
		const uint8_t *dp = data;
		while (pt->letter != 0 && pm != 0)
		{
			if ((pm & 1) != 0)
			{
				switch (pt->type)
				{
				case ParamDescriptor::uint32:	debugPrintf(" %c%" PRIu32, pt->letter, *(const uint32_t*)dp); dp += sizeof(uint32_t); break;
				case ParamDescriptor::int32:	debugPrintf(" %c%" PRIi32, pt->letter, *(const int32_t*)dp); dp += sizeof(int32_t); break;

				case ParamDescriptor::uint16:
				case ParamDescriptor::pwmFreq:	debugPrintf(" %c%u", pt->letter, *(const uint16_t*)dp); dp += sizeof(uint16_t); break;

				case ParamDescriptor::int16:	debugPrintf(" %c%d", pt->letter, *(const uint16_t*)dp); dp += sizeof(int16_t); break;
				case ParamDescriptor::uint8:	debugPrintf(" %c%u", pt->letter, *(const uint8_t*)dp); dp += sizeof(uint8_t); break;
				case ParamDescriptor::int8:		debugPrintf(" %c%d", pt->letter, *(const int8_t*)dp); dp += sizeof(int8_t); break;
				case ParamDescriptor::float_p:	debugPrintf(" %c%.2f", pt->letter, (double)*(const float*)dp); dp += sizeof(float); break;
				case ParamDescriptor::char_p:	debugPrintf(" %c\"%c\"", pt->letter, *(const char*)dp); dp += sizeof(char); break;

				case ParamDescriptor::string:
				case ParamDescriptor::reducedString:
					debugPrintf(" %c\"%s\"", pt->letter, (const char*)dp); dp += strlen((const char*)dp) + 1;
					break;

				default:
					debugPrintf(" %c<unknown format>", pt->letter);
					break;
				}
			}
			pm >>= 1;
			++pt;
		}
		debugPrintf("\n");
	}
}

// End
