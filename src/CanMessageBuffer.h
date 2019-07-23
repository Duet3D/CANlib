/*
 * CanMessageBuffer.h
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEBUFFER_H_
#define SRC_CAN_CANMESSAGEBUFFER_H_

#include <cstdint>
#include <cstddef>

#include "CanId.h"
#include "CanMessageFormats.h"

extern CanAddress GetCanAddress();

// Can message buffer management
class CanMessageBuffer
{
public:
	CanMessageBuffer(CanMessageBuffer *prev) : next(prev) { }

	static void Init(unsigned int numCanBuffers);
	static CanMessageBuffer *Allocate();
	static void Free(CanMessageBuffer*& buf);
	static unsigned int FreeBuffers() { return numFree; }

	// Set up a message buffer to carry a particular message type, setting the priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	CanMessageGeneric *SetupGenericMessage(CanMessageType msgType, uint16_t dest, unsigned int dataLen)
	{
		id.SetRequest(msgType, GetCanAddress(), dest);
		dataLength = dataLen;
		isTimeSyncMessage = false;
		return &msg.generic;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupRequestMessage(uint16_t dest)
	{
		id.SetRequest(T::messageType, GetCanAddress(), dest);
		dataLength = sizeof(T);
		isTimeSyncMessage = false;
		return reinterpret_cast<T*>(&msg);
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupResponseMessage(uint16_t dest)
	{
		id.SetResponse(T::messageType, GetCanAddress(), dest);
		dataLength = sizeof(T);
		isTimeSyncMessage = false;
		return reinterpret_cast<T*>(&msg);
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupBroadcastMessage(bool isTimeSync = false)
	{
		id.SetBroadcast(T::messageType, GetCanAddress());
		dataLength = sizeof(T);
		isTimeSyncMessage = isTimeSync;
		return reinterpret_cast<T*>(&msg);
	}

	CanMessageBuffer *next;
	CanId id;
	size_t dataLength;
	CanMessage msg;
	bool isTimeSyncMessage;

	static constexpr uint16_t broadcastId = 63;

private:
	static CanMessageBuffer *freelist;
	static unsigned int numFree;
};

#endif /* SRC_CAN_CANMESSAGEBUFFER_H_ */
