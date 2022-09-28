/*
 * GCodeResult.h
 *
 *  Created on: 1 Oct 2017
 *      Author: David
 */

#ifndef SRC_GCODERESULT_H_
#define SRC_GCODERESULT_H_

#include <cstdint>

// Enumeration to specify the result of attempting to process a GCode command
enum class GCodeResult : uint8_t
{
	notFinished,					// we haven't finished processing this command
	ok,								// we have finished processing this code in the current state, and if the GCodeState is 'normal' then we have finished it completely
	warning,						// the command succeeded but a warning was generated
	warningNotSupported,			// the command is not supported, but for this command we issue a warning not an error
	error,
	errorNotSupported,
	notSupportedInCurrentMode,
	badOrMissingParameter,
	remoteInternalError,
	m291Cancelled
};

// Test whether the command succeeded
inline constexpr bool Succeeded(GCodeResult rslt) noexcept
{
	return rslt == GCodeResult::ok || rslt == GCodeResult::warning;
}

// Convert a true/false error/no-error indication to a GCodeResult
inline constexpr GCodeResult GetGCodeResultFromError(bool err) noexcept
{
	return (err) ? GCodeResult::error : GCodeResult::ok;
}

// Convert a true/false success/failure indication to a GCodeResult
inline constexpr GCodeResult GetGCodeResultFromSuccess(bool ok) noexcept
{
	return (ok) ? GCodeResult::ok : GCodeResult::error;
}

// Convert a true/false finished/not-finished indication to a GCodeResult
inline constexpr GCodeResult GetGCodeResultFromFinished(bool finished) noexcept
{
	return (finished) ? GCodeResult::ok : GCodeResult::notFinished;
}

#endif /* SRC_GCODERESULT_H_ */
