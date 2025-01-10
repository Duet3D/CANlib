/*
 * RemoteInputHandle.h
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#ifndef SRC_REMOTEINPUTHANDLE_H_
#define SRC_REMOTEINPUTHANDLE_H_

// Type used to represent a handle to a remote input
union __attribute__((packed)) RemoteInputHandle
{
	struct __attribute__((packed)) HandleParts
	{
		uint16_t minor : 6,						// endstop switch number within axis (for endstops)
				major : 6,						// axis number (for endstops), or GPIn number, or Z probe number
				type : 4;						// what the handle is used for, see list of types below
		constexpr HandleParts(uint8_t p_type, uint8_t p_major, uint8_t p_minor): minor(p_minor), major(p_major), type(p_type)  { }
	} parts;
	uint16_t all;

	constexpr RemoteInputHandle() : all(0) { }
	constexpr RemoteInputHandle(uint8_t p_type, uint8_t p_major, uint8_t p_minor) : parts(p_type, p_major, p_minor) { }

	constexpr void Set(uint8_t p_type, uint8_t p_major, uint8_t p_minor) noexcept { parts.type = p_type; parts.major = p_major; parts.minor = p_minor; }
	constexpr void Set(uint16_t p_all) noexcept { all = p_all; }
	constexpr uint16_t asU16() const noexcept { return all; }
	constexpr bool IsValid() const noexcept { return parts.type > typeUnset && parts.type < lowestBadType; }
	constexpr bool operator==(RemoteInputHandle other) const noexcept { return all == other.all; }

	static constexpr uint16_t typeUnset = 0, typeEndstop = 1, typeGpIn = 2, typeZprobe = 3, typeAte = 4, typeStallEndstop = 5, lowestBadType = 6;
};

#endif /* SRC_REMOTEINPUTHANDLE_H_ */
