/*
 * CanMessageBuffer.h
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEBUFFER_H_
#define SRC_CAN_CANMESSAGEBUFFER_H_

#include <ecv_duet3d.h>
#include <cstdint>
#include <cstddef>
#include <new>

#include "CanId.h"
#include "CanMessageFormats.h"

class TaskBase;

// The client project must provide function MessageBufferAlloc and MessageBufferDelete
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept;
void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept;

// Can message buffer management
class CanMessageBuffer
{
public:
	CanMessageBuffer() noexcept : next(nullptr), managed(false) { }

	// Replacement new/delete functions, to allocate the memory permanently and avoid the additional RAM needed by malloc
	void* operator new(size_t count) { return MessageBufferAlloc(count, static_cast<std::align_val_t>(alignof(CanMessageBuffer))); }
	void* operator new(size_t count, std::align_val_t align) { return MessageBufferAlloc(count, align); }
	void operator delete(void* ptr) noexcept { MessageBufferDelete(ptr, static_cast<std::align_val_t>(alignof(CanMessageBuffer))); }
	void operator delete(void* ptr, std::align_val_t align) noexcept { MessageBufferDelete(ptr, align); }

	static void Init(unsigned int numCanBuffers) noexcept;
	static CanMessageBuffer *Allocate() noexcept;

#ifdef RTOS
	// Wait for a buffer until one is available. Only one task may call this!
	static CanMessageBuffer *BlockingAllocate() noexcept;
#endif

	static void Free(CanMessageBuffer*_ecv_null & buf) noexcept;
	static unsigned int GetFreeBuffers() noexcept { return numFree; }
	static unsigned int GetAndClearMinFreeBuffers() noexcept
	{
		const unsigned int ret = minNumFree;
		minNumFree = numFree;
		return ret;
	}

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
		reportInFifo = 0;
		spare = 0;
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
		reportInFifo = 0;
		spare = 0;
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
		reportInFifo = 0;
		spare = 0;
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
		reportInFifo = 0;
		spare = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->SetRequestId(rid);
		return rslt;
	}

	// Set up a message buffer to carry a particular message type, setting the dataLength, priority and code fields.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupResponseMessageNoRid(CanAddress src, CanAddress dest) noexcept
	{
		id.SetResponse(T::messageType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		reportInFifo = 0;
		spare = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->ClearReservedFields();
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
		reportInFifo = 0;
		spare = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->ClearReservedFields();
		return rslt;
	}

	// Set up a message buffer to carry a particular non-broadcast message having no request ID, setting the dataLength, priority and code fields.
	// Used to set up non-broadcast status messages and commands that do not require a response, e.g. heater feedforward.
	// Return a pointer to the message data cast to the requested type.
	// Class T must be one of the supported CAN message types.
	template<class T> T* SetupRequestMessageNoRid(CanAddress src, CanAddress dest) noexcept
	{
		id.SetRequest(T::messageType, src, dest);
		dataLength = sizeof(T);
		marker = 0;
		extId = 1;
		fdMode = 1;
		useBrs = 0;
		remote = 0;
		reportInFifo = 0;
		spare = 0;
		T* rslt = reinterpret_cast<T*>(&msg);
		rslt->ClearReservedFields();
		return rslt;
	}

	void DebugPrint(const char *_ecv_array prefix) noexcept;

	CanMessageBuffer *_ecv_null next;
	CanId id;
	size_t dataLength;
	uint16_t timeStamp;
	uint16_t marker : 8,			// message marker for transmit messages
			extId : 1,				// true to send this using an extended ID
			fdMode : 1,				// true to send as CAN-FD, false for plain CAN
			useBrs : 1,				// true to use bit rate switching (only for CAN-FD)
			remote : 1,				// true to set the 'remote' bit in the frame
			reportInFifo : 1,		// true to report transmission complete via TxEventFifo
			spare : 2,				// spare bits that are cleared by the Setup calls but are otherwise not used
			managed : 1;			// true if this buffer is allocated from heap or freelist and returned to freelist, false if it is a local buffer
	CanMessage msg;

private:
	static CanMessageBuffer *_ecv_null volatile freelist;
	static volatile unsigned int numFree;
	static volatile unsigned int minNumFree;

#ifdef RTOS
	static TaskBase *_ecv_from _ecv_null volatile bufferWaitingTask;
#endif

	CanMessageBuffer(CanMessageBuffer *prev) noexcept : next(prev), managed(true) { }
};

// Helper class to manage CAN message buffer pointers, to ensure they get released if an exception occurs
class CanMessageBufferHandle
{
public:
	CanMessageBufferHandle(CanMessageBuffer *b) : buf(b) { }
	~CanMessageBufferHandle() { if (buf != nullptr) { CanMessageBuffer::Free(buf); } }

	CanMessageBuffer *_ecv_null Access() const { return buf; }
	CanMessageBuffer *_ecv_null HandOver() { CanMessageBuffer *_ecv_null ret = buf; buf = nullptr; return ret; }

private:
	CanMessageBuffer *_ecv_null buf;
};

#endif /* SRC_CAN_CANMESSAGEBUFFER_H_ */
