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
#include <new>

#include "CanId.h"
#include "CanMessageFormats.h"

// The client project must provide function MessageBufferAlloc and MessageBufferDelete
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept;
void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept;

// Can message buffer management
class CanMessageBuffer
{
public:
	CanMessageBuffer(CanMessageBuffer *prev) noexcept : next(prev) { }

	// Replacement new/delete functions, to allocate the memory permanently and avoid the additional RAM needed by malloc
	void* operator new(size_t count) { return MessageBufferAlloc(count, static_cast<std::align_val_t>(alignof(CanMessageBuffer))); }
	void* operator new(size_t count, std::align_val_t align) { return MessageBufferAlloc(count, align); }
	void operator delete(void* ptr) noexcept { MessageBufferDelete(ptr, static_cast<std::align_val_t>(alignof(CanMessageBuffer))); }
	void operator delete(void* ptr, std::align_val_t align) noexcept { MessageBufferDelete(ptr, align); }

	static void Init(unsigned int numCanBuffers) noexcept;
	static CanMessageBuffer *Allocate() noexcept;
	static void Free(CanMessageBuffer*& buf) noexcept;
	static unsigned int FreeBuffers() noexcept { return numFree; }
	static unsigned int MinFreeBuffers() noexcept { return minNumFree; }

	// Set up a message buffer to carry a particular message type, setting the priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	CanMessageGeneric *SetupGenericRequestMessage(CanRequestId rid, CanAddress src, CanAddress dest, CanMessageType msgType, unsigned int dataLen) noexcept
	{
		id.SetRequest(msgType, src, dest);
		dataLength = dataLen;
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		msg.generic.requestId = rid;
		return &msg.generic;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupRequestMessage(CanRequestId rid, CanAddress src, CanAddress dest) noexcept
	{
		id.SetRequest(T::messageType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->SetRequestId(rid);
		return rslt;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupRequestMessage(CanRequestId rid, CanAddress src, CanAddress dest, CanMessageType msgType) noexcept
	{
		id.SetRequest(msgType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->SetRequestId(rid);
		return rslt;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupResponseMessage(CanRequestId rid, CanAddress src, CanAddress dest) noexcept
	{
		id.SetResponse(T::messageType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->SetRequestId(rid);
		return rslt;
	}

	// Set up a message buffer to carry a particular broadcast message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupBroadcastMessage(CanAddress src) noexcept
	{
		id.SetBroadcast(T::messageType, src);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		return reinterpret_cast<T*>(&msg);
	}

	// Set up a message buffer to carry a particular non-broadcast status message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupStatusMessage(CanAddress src, CanAddress dest) noexcept
	{
		id.SetRequest(T::messageType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		return reinterpret_cast<T*>(&msg);
	}

	void DebugPrint(const char *prefix) noexcept;

	CanMessageBuffer *next;
	CanId id;
	size_t dataLength;
	uint16_t timeStamp;
	uint16_t marker : 8,			// message marker for transmit messages
			extId : 1,
			fdMode : 1,
			useBrs : 1,
			remote : 1;
	CanMessage msg;

private:
	static CanMessageBuffer * volatile freelist;
	static volatile unsigned int numFree;
	static volatile unsigned int minNumFree;
};

// Helper class to manage CAN message buffer pointers, to ensure they get released if an exception occurs
class CanMessageBufferHandle
{
public:
	CanMessageBufferHandle(CanMessageBuffer *b) : buf(b) { }
	~CanMessageBufferHandle() { if (buf != nullptr) { CanMessageBuffer::Free(buf); } }

	CanMessageBuffer *Access() const { return buf; }
	CanMessageBuffer *HandOver() { CanMessageBuffer *ret = buf; buf = nullptr; return ret; }

private:
	CanMessageBuffer *buf;
};

#endif /* SRC_CAN_CANMESSAGEBUFFER_H_ */
