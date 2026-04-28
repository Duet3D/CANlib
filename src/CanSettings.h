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

#include <ecv_duet3d.h>
#include "CanId.h"
#include <General/SimpleMath.h>

// In the following structure, the time quantum is 1 cycle of the 48MHz CAN clock that is used on all types of Duet 3 expansion and tool board.
// The tseg1 field excludes the 1-clock sync phase for historical reasons. We retain it for compatibility with existing date stored in NVRAM.
// The CAN bit time is tseg1 + tseg2 + 1 time quanta, so for the default bit rate of 1Mbit/sec this must add up to 48.
// The storage of the data rate parameters when bit rate switching is used is a little odd because they were originally not stored and only two 16-bit spare words were provided.
// They are done in a way that preserves compatibility with parameters already stored in NVRAM when BRS is not used.
struct CanTiming
{
	uint16_t period;					// number of time quanta in 1 bit time, or 0xFFFF if this and the following fields have not been set
	uint16_t nTseg1;					// how far into the bit period the sample point is (minimum 1, maximum period-2) less 1
	uint16_t nJumpWidth;				// the (re)synchronisation jump width during the arbitration and CRC phases. The maximum is (period - (tseg1 + 1) but we check that when we program the CAN peripheral.
	uint16_t dataRateMultiplier : 4,	// how many times faster the data bit rate is (must be 1, 2, 3, 4, 6 or 8) minus 1. 0 or 0x0F mean don't use BRS.
			 dTseg1 : 8,				// number of time quanta before the data phase sample point, less 1
			 spare1 : 4;
	uint16_t dJumpWidth: 8,				// the jump width during the data phase
			 spare2 : 8;				// make up to 10 bytes for future expansion

	static constexpr uint32_t ClockFrequency = 48'000'000;					// CAN clock used by all Duet 3 boards
	static constexpr uint32_t DefaultCanBitRate = 1'000'000;
	static constexpr float DefaultNormalSamplePoint = 0.78;					// how far we sample into the bit during the arbitration and CRC phases
	static constexpr float DefaultDataSamplePoint = 0.50;					// how far we sample into the bit during the data phase when BRS is used

	constexpr bool IsValid() const noexcept
	{
		return period >= 24 && period <= 4800
			&& nTseg1 != 0 && nTseg1 <= period - 2u
			&& nTseg1 + nJumpWidth + 1 <= period;
	}

	constexpr bool IsUsingBrs() const noexcept
	{
		return dataRateMultiplier != 0 && dataRateMultiplier != 0x0F;
	}

	// Set the bit rate to the requested value, set the sample point and jump width to default values, and disable BRS.
	// This is called by the bootloader, so it must not use any run-time floating point maths in order to keep the SAMC21 bootloader small.
	constexpr void SetDefaults(uint32_t bitRate) noexcept
	{
		constexpr uint32_t DefaultNormalSamplePointTimes1024 = (uint32_t)(DefaultNormalSamplePoint * 1024);

		period = (uint16_t)((ClockFrequency + (bitRate/2))/bitRate);
		nTseg1 = (uint16_t)((period * DefaultNormalSamplePointTimes1024)/1024) - 1;
		nJumpWidth = period - (nTseg1 + 1);									// this is the maximum possible, as recommended by CiA
		dataRateMultiplier = 0x0F;											// disable BRS
	}

	// Set the arbitration phase sample point and set maximum jump width. The period must be set first.
	constexpr void SetNormalSamplePoint(float samplePoint) noexcept
	{
		nTseg1 = (uint16_t)(period * samplePoint) - 1;						// tseg1 excludes the 1-clock sync phase for historical reasons, hence the -1
		nJumpWidth = period - (nTseg1 + 1);
	}

	// Set the arbitration phase jump width. The bit rate and sample point must be set first.
	constexpr void SetNormalJumpWidth(float jw) noexcept
	{
		nJumpWidth = constrain<uint16_t>((uint16_t)(period * jw), 1, period - (nTseg1 + 1));
	}

	// Enable bit rate switching and set the default data phase sample point and jump width
	constexpr void EnableBrs(uint8_t bitRateMultiplier) noexcept
	{
		constexpr uint32_t DefaultDataSamplePointTimes1024 = (uint32_t)(DefaultDataSamplePoint * 1024);
		const uint16_t dataBitPeriod = period/bitRateMultiplier;
		dataRateMultiplier = bitRateMultiplier - 1;
		dTseg1 = (uint16_t)((dataBitPeriod * DefaultDataSamplePointTimes1024)/1024) - 1;
		dJumpWidth = dataBitPeriod - (dTseg1 + 1);
	}

	// Set the data phase sample point and set maximum jump width. The period must be set first.
	constexpr void SetDataSamplePoint(float samplePoint) noexcept
	{
		const uint16_t dataBitPeriod = period/(dataRateMultiplier + 1);
		dTseg1 = (uint16_t)(dataBitPeriod * samplePoint) - 1;				// tseg1 excludes the 1-clock sync phase for historical reasons, hence the -1
		dJumpWidth = dataBitPeriod - (dTseg1 + 1);
	}

	// Set the data phase sample point directly and set maximum jump width
	constexpr void SetDataSamplePointDirect(uint16_t samplePoint) noexcept
	{
		const uint16_t dataBitPeriod = period/(dataRateMultiplier + 1);
		dTseg1 = samplePoint;				// tseg1 excludes the 1-clock sync phase for historical reasons, hence the -1
		dJumpWidth = dataBitPeriod - (dTseg1 + 1);
	}

	// Set the data phase jump width. The bit rate and sample point must be set first.
	constexpr void SetDataJumpWidth(float jw) noexcept
	{
		const uint16_t dataBitPeriod = period/(dataRateMultiplier + 1);
		dJumpWidth = constrain<uint16_t>((uint16_t)(dataBitPeriod * jw), 1, dataBitPeriod - (dTseg1 + 1));
	}
};

// This is read from the user area flash, so all values default to all bits set
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
			fastParametersNotSet : 1,			// set if BRS parameters have not been set
			spare : 13;							// spare bits, must be set to 1 for future compatibility
	uint8_t canAddress;							// the CAN address of this board, or 0xFF if it has not been set
	uint8_t invertedCanAddress;					// the inverted CAN address of this board, or 0xFF if it has not been set
	CanTiming timing;							// this is 8 bytes long
	uint16_t checksum;							// checksum word to make the XOR of all eight 16-bit words the magic value
};

static_assert(sizeof(CanUserAreaData) == 16);

// Where we store the CAN data
constexpr uint32_t CanUserAreaDataOffset_SAME5x = 512 - sizeof(CanUserAreaData);
constexpr uint32_t CanUserAreaDataOffset_SAMC21 = 256 - sizeof(CanUserAreaData);

#endif /* SRC_CANTIMINGDATA_H_ */
