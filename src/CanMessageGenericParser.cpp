/*
 * CanMessageGenericParser.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "CanMessageGenericParser.h"
#include <General/Portability.h>
#include <General/StringRef.h>
#include <General/SimpleMath.h>

bool CanMessageGenericParser::GetStringParam(char c, const StringRef& v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	if (type == ParamDescriptor::ParamType::string || type == ParamDescriptor::ParamType::reducedString)
	{
		v.copy((const char *)msg.data + pos);
		return true;
	}
	return false;
}

// Look for the specified parameter. If found, return its type and set the index of the corresponding data.
ParamDescriptor::ParamType CanMessageGenericParser::FindParameter(char c, unsigned int& pos) const noexcept
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
			if (size == 0)
			{
				// The only item with size 0 is string, so skip up to and including the null terminator
				do
				{
				} while (msg.data[pos++] != 0);
			}
			else if ((d->type & ParamDescriptor::ParamType::isArray) != 0)
			{
				const uint8_t numElems = msg.data[pos++];
				pos += numElems * size;
			}
			else
			{
				pos += size;
			}
		}
		paramMap >>= 1;
	}
	return ParamDescriptor::ParamType::none;
}

bool CanMessageGenericParser::GetUintParam(char c, uint32_t& v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	switch (type)
	{
	case ParamDescriptor::ParamType::uint32:
		v = LoadLEU32(msg.data + pos);
		return true;

	case ParamDescriptor::ParamType::uint16:
	case ParamDescriptor::ParamType::pwmFreq:
		v = LoadLEU16(msg.data + pos);
		return true;

	case ParamDescriptor::ParamType::uint8:
	case ParamDescriptor::ParamType::localDriver:
		v = msg.data[pos];
		return true;

	default:
		return false;
	}
}

bool CanMessageGenericParser::GetUintParam(char c, uint16_t& v) const noexcept
{
	uint32_t v2;
	const bool ret = GetUintParam(c, v2);
	if (ret)
	{
		v = (uint16_t)v2;
	}
	return ret;
}

bool CanMessageGenericParser::GetUintParam(char c, uint8_t& v) const noexcept
{
	uint32_t v2;
	const bool ret = GetUintParam(c, v2);
	if (ret)
	{
		v = (uint8_t)v2;
	}
	return ret;
}

bool CanMessageGenericParser::GetIntParam(char c, int32_t& v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	switch (type)
	{
	case ParamDescriptor::ParamType::int32:
		v = LoadLEI32(msg.data + pos);
		return true;

	case ParamDescriptor::ParamType::int16:
		v = LoadLEI16(msg.data + pos);
		return true;

	case ParamDescriptor::ParamType::int8:
		v = *reinterpret_cast<const int8_t*>(msg.data + pos);
		return true;

	default:
		return false;
	}
}

bool CanMessageGenericParser::GetIntParam(char c, int16_t& v) const noexcept
{
	int32_t v2;
	const bool ret = GetIntParam(c, v2);
	if (ret)
	{
		v = (int16_t)v2;
	}
	return ret;
}

bool CanMessageGenericParser::GetIntParam(char c, int8_t& v) const noexcept
{
	int32_t v2;
	const bool ret = GetIntParam(c, v2);
	if (ret)
	{
		v = (int8_t)v2;
	}
	return ret;
}

bool CanMessageGenericParser::GetFloatParam(char c, float& v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	switch (type)
	{
	case ParamDescriptor::ParamType::float_p:
		v = LoadLEF32(msg.data + pos);
		return true;

	case ParamDescriptor::ParamType::float16_p:
		v = LoadLEF16(msg.data + pos);
		return true;

	default:
		return false;
	}
}

bool CanMessageGenericParser::GetCharParam(char c, char& v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	switch (type)
	{
	case ParamDescriptor::ParamType::char_p:
		v = *reinterpret_cast<const char*>(msg.data + pos);
		return true;

	default:
		return false;
	}
}

bool CanMessageGenericParser::GetBoolParam(char c, bool &v) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	switch (type)
	{
	case ParamDescriptor::ParamType::int32:
		v = (*reinterpret_cast<const int32_t*>(msg.data + pos) > 0);
		return true;

	case ParamDescriptor::ParamType::int16:
		v = (*reinterpret_cast<const int16_t*>(msg.data + pos) > 0);
		return true;

	case ParamDescriptor::ParamType::int8:
		v = (*reinterpret_cast<const int8_t*>(msg.data + pos) > 0);
		return true;

	case ParamDescriptor::ParamType::uint32:
		v = (*reinterpret_cast<const uint32_t*>(msg.data + pos) != 0);
		return true;

	case ParamDescriptor::ParamType::uint16:
		v = (*reinterpret_cast<const uint16_t*>(msg.data + pos) != 0);
		return true;

	case ParamDescriptor::ParamType::uint8:
		v = (*reinterpret_cast<const uint8_t*>(msg.data + pos) != 0);
		return true;

	default:
		return false;
	}
}

bool CanMessageGenericParser::GetArrayParam(char c, ParamDescriptor::ParamType pt, size_t& numValues, const uint8_t*& values) const noexcept
{
	unsigned int pos;
	const ParamDescriptor::ParamType type = FindParameter(c, pos);
	if (type == pt)
	{
		if (numValues > msg.data[pos])
		{
			numValues = msg.data[pos];
		}
		values = msg.data + pos + 1;
		return true;
	}
	return false;
}

bool CanMessageGenericParser::GetUint16ArrayParam(char c, size_t& numValues, uint16_t *values) const noexcept
{
	const uint8_t *p;
	if (GetArrayParam(c, ParamDescriptor::ParamType::uint16_array, numValues, p))
	{
		for (size_t i = 0; i < numValues; ++i)
		{
			*values++ = LoadLEU16(p);
			p += sizeof(uint16_t);
		}
		return true;
	}
	return false;
}

// Get a float array parameter. We copy the float array to a user array because the array in the message may be misaligned.
// On entry, numValues is the size of the user array
// On return, numValue is the number passed in the message
bool CanMessageGenericParser::GetFloatArrayParam(char c, size_t& numValues, float *values) const noexcept
{
	const uint8_t *p;
	if (GetArrayParam(c, ParamDescriptor::ParamType::float_array, numValues, p))
	{
		for (size_t i = 0; i < numValues; ++i)
		{
			*values++ = LoadLEF32(p);
			p += sizeof(float);
		}
		return true;
	}
	return false;
}

bool CanMessageGenericParser::HasParameter(char c) const noexcept
{
	unsigned int pos;
	return FindParameter(c, pos) != ParamDescriptor::ParamType::none;
}

// End
