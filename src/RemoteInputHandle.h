/*
 * RemoteInputHandle.h
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#ifndef SRC_REMOTEINPUTHANDLE_H_
#define SRC_REMOTEINPUTHANDLE_H_

// Type used to represent a handle to a remote input
struct __attribute__((packed)) RemoteInputHandle
{
	RemoteInputHandle(uint8_t p_type, uint8_t p_major, uint8_t p_minor) noexcept { Set(p_type, p_major, p_minor); }
	RemoteInputHandle() noexcept { u.all = 0; }		// constructor sets it to invalid
	void Set(uint8_t p_type, uint8_t p_major, uint8_t p_minor) noexcept { u.parts.type = p_type; u.parts.major = p_major; u.parts.minor = p_minor; }
	void Set(uint16_t p_all) noexcept { u.all = p_all; }
	uint16_t asU16() const noexcept { return u.all; }
	bool IsValid() const noexcept { return u.parts.type > typeUnset && u.parts.type < lowestBadType; }

	union
	{
		struct
		{
			uint16_t minor : 8,						// endstop switch number within axis (for endstops)
					major : 4,						// axis number (for endstops), or GPIn number, or Z probe number
					type : 4;
		} parts;
		uint16_t all;
	} u;

	static constexpr uint16_t typeUnset = 0, typeEndstop = 1, typeGpIn = 2, typeZprobe = 3, typeAte = 4, lowestBadType = 5;
};

#endif /* SRC_REMOTEINPUTHANDLE_H_ */
