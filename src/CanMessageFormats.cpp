/*
 * CanMessageFormats.cpp
 *
 *  Created on: 15 Jul 2019
 *      Author: David
 */

#include "CanMessageFormats.h"
#include <cinttypes>

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

void CanMessageMovement::DebugPrint()
{
	debugPrintf("Can: %08" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %.2f %.2f:",
		whenToExecute, accelerationClocks, steadyClocks, decelClocks, (double)initialSpeedFraction, (double)finalSpeedFraction);
	for (size_t i = 0; i < MaxDriversPerCanSlave; ++i)
	{
		debugPrintf(" %" PRIi32, perDrive[i].steps);
	}
	debugPrintf("\n");
}

// End
