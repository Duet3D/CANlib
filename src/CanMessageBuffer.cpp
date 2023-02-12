/*
 * CanMessageBuffer.cpp
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#include "CanMessageBuffer.h"
#include "RTOSIface/RTOSIface.h"
#include <cinttypes>

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

CanMessageBuffer * volatile CanMessageBuffer::freelist = nullptr;
volatile unsigned int CanMessageBuffer::numFree = 0;
volatile unsigned int CanMessageBuffer::minNumFree = 0;

#ifdef RTOS
TaskBase * volatile CanMessageBuffer::bufferWaitingTask = nullptr;
#endif

void CanMessageBuffer::Init(unsigned int numCanBuffers) noexcept
{
	freelist = nullptr;
	while (numCanBuffers != 0)
	{
		freelist = new CanMessageBuffer(freelist);
		--numCanBuffers;
		++numFree;
	}
	minNumFree = numFree;
}

CanMessageBuffer *CanMessageBuffer::Allocate() noexcept
{
	TaskCriticalSectionLocker lock;

	CanMessageBuffer *ret = freelist;
	if (ret != nullptr)
	{
		freelist = ret->next;
		ret->next = nullptr;
		--numFree;
		if (numFree < minNumFree)
		{
			minNumFree = numFree;
		}
	}
	return ret;
}

#ifdef RTOS

// Wait for a buffer until one is available. Only one task may call this!
CanMessageBuffer *CanMessageBuffer::BlockingAllocate() noexcept
{
	while (true)
	{
		{
			TaskCriticalSectionLocker lock;

			CanMessageBuffer *ret = freelist;
			if (ret != nullptr)
			{
				freelist = ret->next;
				ret->next = nullptr;
				--numFree;
				if (numFree < minNumFree)
				{
					minNumFree = numFree;
				}
				return ret;
			}

			bufferWaitingTask = TaskBase::GetCallerTaskHandle();
		}
		TaskBase::Take();
	}
}

#endif

void CanMessageBuffer::Free(CanMessageBuffer*& buf) noexcept
{
	if (buf != nullptr && buf->managed)
	{
		TaskCriticalSectionLocker lock;
		buf->next = freelist;
		freelist = buf;
		buf = nullptr;
		++numFree;
#ifdef RTOS
		TaskBase * const waitingTask = bufferWaitingTask;
		if (waitingTask != nullptr)
		{
			bufferWaitingTask = nullptr;
			waitingTask->GiveFromISR();
		}
#endif
	}
}

void CanMessageBuffer::DebugPrint(const char *prefix) noexcept
{
	debugPrintf("%s%08" PRIx32 " %02x %02x %02x %02x %02x %02x %02x %02x\n", prefix, id.GetWholeId(), msg.raw[0], msg.raw[1], msg.raw[2], msg.raw[3], msg.raw[4], msg.raw[5], msg.raw[6], msg.raw[7]);
}

// End
