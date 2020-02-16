/*
 * Duet3Limits.h
 *
 * This file defines the system-wide limits for Duet 3 that need to be known by main boards and expansion boards
 *
 *  Created on: 27 Dec 2019
 *      Author: David
 */

#ifndef SRC_DUET3LIMITS_H_
#define SRC_DUET3LIMITS_H_

#include <cstdint>
#include <cstddef>

constexpr size_t MaxSensors = 56;						// limited by the size of bitmap we can store in an ExpressionValue
constexpr size_t MaxHeaters = 32;
constexpr size_t MaxMonitorsPerHeater = 3;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxZProbeProgramBytes = 8;				// maximum number of bytes in a Z probe program

constexpr size_t MaxFans = 16;

constexpr size_t MaxGpOutPorts = 16;

// The following currently don't need to be know by expansion boards, but might in future
constexpr size_t MaxGpInPorts = 16;
constexpr size_t MaxSpindles = 2;						// maximum number of configurable spindles

#endif /* SRC_DUET3LIMITS_H_ */
