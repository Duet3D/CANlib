/*
 * CanMessageGenericParse.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "CanMessageGenericParser.h"

bool CanMessageGenericParser::GetStringParam(char c, const StringRef& v) const
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	if (type == ParamDescriptor::ParamType::string)
	{
		v.copy((const char *)msg.data + pos);
		return true;
	}
	return false;
}

bool CanMessageGenericParser::GetNonStringParam(char c, char *v, ParamDescriptor::ParamType expectedType) const
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	if (type == expectedType)
	{
		memcpy(v, msg.data + pos, expectedType & 0x0F);
		return true;
	}
	return false;
}

// Look for the specified parameter. If found, return its type and set the index of the corresponding data.
ParamDescriptor::ParamType CanMessageGenericParser::FindParameter(char c, unsigned int& pos) const
{
	pos = 0;
	uint32_t paramMap = msg.paramMap;
	for (const ParamDescriptor *d = paramTable; d->letter != 0 && paramMap != 0; ++d)
	{
		const bool present = (paramMap & 1) != 0;
		if (d->letter == c)
		{
			return (present) ? d->type : ParamDescriptor::ParamType::none;
		}
		if (present)
		{
			// This parameter is present, so skip it
			unsigned int size = d->ItemSize();
			if (size != 0)
			{
				pos += size;
			}
			else
			{
				// The only item with size 0 is string, so skip up to and including the null terminator
				do
				{
				} while (msg.data[pos++] != 0);
			}
		}
		paramMap >>= 1;
	}
	return ParamDescriptor::ParamType::none;
}

// End

