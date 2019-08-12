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
	CanMessageGeneric *SetupGenericMessage(CanMessageType msgType, CanAddress src, CanAddress dest, unsigned int dataLen)
	{
		id.SetRequest(msgType, src, dest);
		dataLength = dataLen;
		return &msg.generic;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupRequestMessage(CanAddress src, CanAddress dest)
	{
		id.SetRequest(T::messageType, src, dest);
		dataLength = sizeof(T);
		return reinterpret_cast<T*>(&msg);
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupResponseMessage(CanAddress src, CanAddress dest)
	{
		id.SetResponse(T::messageType, src, dest);
		dataLength = sizeof(T);
		return reinterpret_cast<T*>(&msg);
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupBroadcastMessage(CanAddress src)
	{
		id.SetBroadcast(T::messageType, src);
		dataLength = sizeof(T);
		return reinterpret_cast<T*>(&msg);
	}

	void DebugPrint(const char *prefix);

	CanMessageBuffer *next;
	CanId id;
	size_t dataLength;
	CanMessage msg;

	static constexpr uint16_t broadcastId = 63;

private:
	static CanMessageBuffer *freelist;
	static unsigned int numFree;
};

#endif /* SRC_CAN_CANMESSAGEBUFFER_H_ */
