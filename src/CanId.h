/*
 * CanId.h
 *
 *  Created on: 5 Jul 2019
 *      Author: David
 */

#ifndef SRC_CAN_CANID_H_
#define SRC_CAN_CANID_H_

#include <cstdint>

// CAN message types. This is a 13-bit field. Low numbers have highest priority.
enum class CanMessageType : uint16_t
{
	emergencyStop = 1000,
	startup = 1001,
	controlledStop = 1002,
	timeSync = 1003,
	movement = 1004,
	m906 = (3u << 10) + 906
};

typedef uint8_t CanAddress;					// only the lower 7 bits are available

// CAN identifier
// A CAN identifier must identify
// 1. The sender, to make sure that 2 nodes can't both try to transmit at the same time.
// 2. The destination, so that each device can use the hardware ID filters in the MCU to receive only messages addressed to it, and broadcast messages.
// 3. The priority, so that if 2 nodes want to transmit at the same time, urgent messages can take precedence over less urgent messages.
// The CAN ID is transmitted most significant bit first. Logic level 0 is dominant. Therefore, lower IDs have greater priority.
// We use 29-bit CAN identifiers, split as for UAVCAN:
// Most significant 13 bits: message type/priority (lower is higher priority)
// Next 1 bit: response not request
// Next 7 bits: source address
// Next 1 bit: unused
// Lowest 7 bits: destination address
class CanId
{
	uint32_t all;

public:
	static constexpr uint32_t DestIdMask = 0x7F;
	static constexpr unsigned int DestIdShift = 0;
	static constexpr unsigned int SrcIdShift = 8;
	static constexpr CanAddress BroadcastId = 0x7F;

	void SetRequest(CanMessageType msgType, CanAddress src, CanAddress dst)
	{
		all = ((uint32_t)msgType << 16) | ((uint32_t)src << 8) | (uint32_t)dst;
	}

	void SetResponse(CanMessageType msgType, CanAddress src, CanAddress dst)
	{
		all = ((uint32_t)msgType << 16) | ((uint32_t)src << 8) | (uint32_t)dst | (1u << 15);
	}

	void SetBroadcast(CanMessageType msgType, CanAddress src)
	{
		all = ((uint32_t)msgType << 16) | ((uint32_t)src << 8) | (uint32_t)BroadcastId;
	}

	void SetReceivedId(uint32_t id)
	{
		all = id;
	}

	uint8_t Src() const { return (all >> SrcIdShift) & DestIdMask; }
	uint8_t Dst() const { return all & DestIdMask; }
	CanMessageType MsgType() const { return (CanMessageType)((all >> 16) & 0x1FFF); }
	uint32_t GetWholeId() const { return all; }
};

#endif /* SRC_CAN_CANID_H_ */
