/*
 * CanMessageBuffer.cpp
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#include "CanMessageBuffer.h"
#include "RTOSIface/RTOSIface.h"
#include "CANlibNotifyIndices.h"
#include <cinttypes>

extern "C" void debugPrintf(const char *_ecv_array fmt, ...) __attribute__ ((format (printf, 1, 2)));

CanMessageBuffer *_ecv_null volatile CanMessageBuffer::freelist = nullptr;
std::atomic<unsigned int> CanMessageBuffer::numFree = 0;
volatile unsigned int CanMessageBuffer::minNumFree = 0;

#ifdef RTOS
TaskBase *_ecv_from _ecv_null volatile CanMessageBuffer::bufferWaitingTask = nullptr;
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

	CanMessageBuffer *_ecv_null ret = freelist;
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
	return _ecv_not_null(ret);
}

#ifdef RTOS

// Wait for a buffer until one is available. Only one task may call this!
CanMessageBuffer *CanMessageBuffer::BlockingAllocate() noexcept
{
	while (true)
	{
		{
			TaskCriticalSectionLocker lock;

			CanMessageBuffer *_ecv_null ret = freelist;
			if (ret != nullptr)
			{
				freelist = ret->next;
				ret->next = nullptr;
				--numFree;
				if (numFree < minNumFree)
				{
					minNumFree = numFree;
				}
				return _ecv_not_null(ret);
			}

			bufferWaitingTask = TaskBase::GetCallerTaskHandle();
		}
		TaskBase::TakeIndexed(NotifyIndices::CanMessageBuffer);
	}
}

#endif

void CanMessageBuffer::Free(CanMessageBuffer*_ecv_null & buf) noexcept
{
	if (buf != nullptr && buf->managed)
	{
		TaskCriticalSectionLocker lock;
		buf->next = freelist;
		freelist = buf;
		buf = nullptr;
		++numFree;
#ifdef RTOS
		TaskBase *_ecv_null const waitingTask = bufferWaitingTask;
		if (waitingTask != nullptr)
		{
			bufferWaitingTask = nullptr;
			waitingTask->GiveFromISR(NotifyIndices::CanMessageBuffer);
		}
#endif
	}
}

void CanMessageBuffer::DebugPrint(const char *_ecv_array prefix) noexcept
{
	debugPrintf("%s%08" PRIx32 " %02x %02x %02x %02x %02x %02x %02x %02x\n", prefix, id.GetWholeId(), msg.raw[0], msg.raw[1], msg.raw[2], msg.raw[3], msg.raw[4], msg.raw[5], msg.raw[6], msg.raw[7]);
}

// End
