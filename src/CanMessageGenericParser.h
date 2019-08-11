/*
 * CanMessageGenericParser.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_CANMESSAGEGENERICPARSER_H_
#define SRC_CANMESSAGEGENERICPARSER_H_

#include "CanMessageFormats.h"

// Class to extract parameters from a generic CAN message
class CanMessageGenericParser
{
public:
	CanMessageGenericParser(const CanMessageGeneric& p_msg, const ParamDescriptor *p_param) : msg(p_msg), paramTable(p_param) { }

	// Methods to extract parameters from a CAN message. Each returns true if the specified parameter was present.
	bool GetUintParam(char c, uint32_t& v) const;
	bool GetUintParam(char c, uint16_t& v) const;
	bool GetUintParam(char c, uint8_t& v) const;
	bool GetIntParam(char c, int32_t& v) const;
	bool GetIntParam(char c, int16_t& v) const;
	bool GetIntParam(char c, int8_t& v) const;
	bool GetFloatParam(char c, float& v) const;
	bool GetCharParam(char c, char& v) const;
	bool GetStringParam(char c, const StringRef& v) const;
	bool GetBoolParam(char c, bool &v) const;
	bool HasParameter(char c) const;

private:
	ParamDescriptor::ParamType FindParameter(char c, unsigned int& pos) const;

	const CanMessageGeneric& msg;
	const ParamDescriptor * const paramTable;
};

#endif /* SRC_CANMESSAGEGENERICPARSER_H_ */
