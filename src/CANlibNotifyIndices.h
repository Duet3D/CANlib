/*
 * CANlibNotifyIndices.h
 *
 *  Created on: 2 Jan 2024
 *      Author: David
 *
 *  Definitions of task notification indices used by CANlib layer
 */

#ifndef SRC_CANLIBNOTIFYINDICES_H_
#define SRC_CANLIBNOTIFYINDICES_H_

#include <CoreNotifyIndices.h>

namespace NotifyIndices
{
	constexpr uint32_t CanMessageBuffer = NextAvailableAfterCore;
	constexpr uint32_t NextAvailableAfterCANlib = CanMessageBuffer + 1;
}

#endif /* SRC_CANLIBNOTIFYINDICES_H_ */
