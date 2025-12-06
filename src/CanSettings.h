/*
 * CanSettings.h
 *
 *  Created on: 18 Dec 2019
 *      Author: David
 *
 *  This structure is saved in the user area of NVRAM on Duet 3 tool and expansion boards to specify the CAN base rate timing data if it has been changed.
 */

#ifndef SRC_CANTIMINGDATA_H_
#define SRC_CANTIMINGDATA_H_

#include "CanId.h"
#include <General/SimpleMath.h>

// In the following structure, the time quantum is 1 cycle of the 48MHz CAN clock that is used on all types of Duet 3 expansion and tool board.
// The tseg1 field excludes the 1-clock sync phase for historical reasons. We retain it for compatibility with existing date stored in NVRAM.
// The CAN bit time is tseg1 + tseg2 + 1 time quanta, so for the default bit rate of 1Mbit/sec this must add up to 48.
struct CanTiming
{
	uint16_t period;				// number of time quanta in 1 bit time, or 0xFFFF if this and the following fields have not been set
	uint16_t tseg1;					// how far into the bit period the sample point is (minimum 1, maximum period-2) less 1
	uint16_t jumpWidth;				// the (re)synchronisation jump width. The maximum is (period - (tseg1 + 1) but we check that when we program the CAN peripheral.

	static constexpr uint32_t ClockFrequency = 48'000'000;					// CAN clock used by all Duet 3 boards
	static constexpr uint32_t DefaultCanBitRate = 1'000'000;
	static constexpr float DefaultSamplePoint = 0.78;						// how far we sample into the bit
	static constexpr float DefaultJumpWidth = 0.25;							// how much of the bit the receive clock can jump to resync. Gets limited when we program the CAN peripheral

	constexpr bool IsValid() const noexcept
	{
		return period >= 24 && period <= 4800
			&& tseg1 != 0 && tseg1 <= period - 2;
	}

	// Set the sample point. The period must be set first.
	constexpr void SetSamplePoint(float samplePoint) noexcept
	{
		tseg1 = (uint16_t)(period * samplePoint) - 1;						// tseg1 excludes the 1-clock sync phase for historical reasons, hence the -1
	}

	// Set the jump width. The bit rate and sample point must be set first.
	constexpr void SetJumpWidth(float jw) noexcept
	{
		jumpWidth = constrain<uint16_t>((uint16_t)(period * jw), 1, period - tseg1 - 1);
	}

	// The following is called by the bootloader, so it must not use any run-time floating point maths in order to keep the SAMC21 bootloader small
	constexpr void SetDefaults(uint32_t bitRate) noexcept
	{
		constexpr uint32_t DefaultSamplePointTimes1024 = (uint32_t)(DefaultSamplePoint * 1024);

		period = (uint16_t)((ClockFrequency + (bitRate/2))/bitRate);
		tseg1 = (uint16_t)((period * DefaultSamplePointTimes1024)/1024) - 1;
		jumpWidth = period - (tseg1 + 1);									// this is the maximum possible, as recommended by CiA
	}
};

// This is read from the user area RAM, so all values default to all bits set
class CanUserAreaData
{
public:
	void Clear() noexcept;
	bool IsValid() const noexcept;
	void SetCanAddress(CanAddress address) noexcept;
	bool AddressValid() const noexcept;
	CanAddress GetCanAddress(CanAddress defaultAddress) const noexcept;
	void SetTiming(const CanTiming& data) noexcept;
	void GetTiming(CanTiming& data) const noexcept;

private:
	uint16_t GetChecksum() const noexcept;
	void UpdateChecksum() noexcept;

	static constexpr uint16_t magic = 0x4321;	// the expected XOR of all eight 16-bit words

	// Total 16 bytes
	uint16_t canIdV1NotSet : 1,					// set if canAddress does not contain the CAN address to use
			timingV1NotSet : 1,					// set if timing has not been set
			spare : 14;							// spare bits, must be set to 1 for future compatibility
	uint8_t canAddress;							// the CAN address of this board, or 0xFF if it has not been set
	uint8_t invertedCanAddress;					// the inverted CAN address of this board, or 0xFF if it has not been set
	CanTiming timing;							// this is 6 bytes long
	uint16_t spare1, spare2;					// make up to 14 bytes for future expansion
	uint16_t checksum;							// checksum word to make the XOR of all eight 16-bit words the magic value
};

static_assert(sizeof(CanUserAreaData) == 16);


// Where we store the CAN data
constexpr uint32_t CanUserAreaDataOffset_SAME5x = 512 - sizeof(CanUserAreaData);
constexpr uint32_t CanUserAreaDataOffset_SAMC21 = 256 - sizeof(CanUserAreaData);

#endif /* SRC_CANTIMINGDATA_H_ */
