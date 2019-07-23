/*
 * CanMessageGenericParse.h
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
	bool GetUint32Param(char c, uint32_t& v) const	{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::uint32); }
	bool GetInt32Param(char c, int32_t& v) const	{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::int32); }
	bool GetFloatParam(char c, float& v) const		{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::float_p); }
	bool GetUint16Param(char c, uint16_t& v) const	{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::uint16); }
	bool GetInt16Param(char c, int32_t& v) const	{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::int16); }
	bool GetUint8Param(char c, uint8_t& v) const	{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::uint8); }
	bool GetInt8Param(char c, int8_t& v) const		{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::int8); }
	bool GetCharParam(char c, char& v) const		{ return GetNonStringParam(c, (char*)&v, ParamDescriptor::ParamType::char_p); }
	bool GetStringParam(char c, const StringRef& v) const;

private:
	bool GetNonStringParam(char c, char *v, ParamDescriptor::ParamType expectedType) const;
	ParamDescriptor::ParamType FindParameter(char c, unsigned int& pos) const;

	const CanMessageGeneric& msg;
	const ParamDescriptor * const paramTable;
};

#endif /* SRC_CANMESSAGEGENERICPARSER_H_ */
