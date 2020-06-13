/*
 * CanMessageGenericParser.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_CANMESSAGEGENERICPARSER_H_
#define SRC_CANMESSAGEGENERICPARSER_H_

#include "CanMessageFormats.h"

class StringRef;

// Class to extract parameters from a generic CAN message
class CanMessageGenericParser
{
public:
	CanMessageGenericParser(const CanMessageGeneric& p_msg, const ParamDescriptor *p_param) noexcept : msg(p_msg), paramTable(p_param) { }

	// Methods to extract parameters from a CAN message. Each returns true if the specified parameter was present.
	bool GetUintParam(char c, uint32_t& v) const noexcept;
	bool GetUintParam(char c, uint16_t& v) const noexcept;
	bool GetUintParam(char c, uint8_t& v) const noexcept;
	bool GetIntParam(char c, int32_t& v) const noexcept;
	bool GetIntParam(char c, int16_t& v) const noexcept;
	bool GetIntParam(char c, int8_t& v) const noexcept;
	bool GetFloatParam(char c, float& v) const noexcept;
	bool GetCharParam(char c, char& v) const noexcept;
	bool GetStringParam(char c, const StringRef& v) const noexcept;
	bool GetBoolParam(char c, bool &v) const noexcept;

	bool GetUint8ArrayParam(char c, size_t& numValues, const uint8_t*& values) noexcept;
	bool GetFloatArrayParam(char c, size_t& numValues, float *values) noexcept;

	bool HasParameter(char c) const noexcept;

private:
	ParamDescriptor::ParamType FindParameter(char c, unsigned int& pos) const noexcept;

	const CanMessageGeneric& msg;
	const ParamDescriptor * const paramTable;
};

#endif /* SRC_CANMESSAGEGENERICPARSER_H_ */
