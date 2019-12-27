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

constexpr size_t MaxSensors = 64;
constexpr size_t MaxHeaters = 16;
constexpr size_t MaxExtraHeaterProtections = 16;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxZProbeProgramBytes = 8;				// maximum number of bytes in a Z probe program

constexpr size_t MaxGpioPorts = 16;
constexpr size_t MaxFans = 16;

#endif /* SRC_DUET3LIMITS_H_ */
