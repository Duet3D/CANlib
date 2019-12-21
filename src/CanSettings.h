/*
 * CanSettings.h
 *
 *  Created on: 18 Dec 2019
 *      Author: David
 *
 *  This structure is saved in the user area of NVRAM to specify the CAN base rate timing data if it has been changed.
 */

#ifndef SRC_CANTIMINGDATA_H_
#define SRC_CANTIMINGDATA_H_

#include <CanId.h>

// In the following structure, the time quantum is 1 cycle of the 48MHz CAN clock that is used on all types of Duet 3 expansion and tool board.
// The default bit timing is: TSEG1 26, period 48, SJW 8. The CAN bit time is NTSEG1 + NTSEG2 + 1 time quanta, so the default bit rate is 1MHz.
// Currently we use a prescaler of 2, so the CAN clock runs at 24MHz and we need to halve these values. But we have the option to switch to 48MHz in future.
struct CanTiming
{
	uint16_t period;				// number of time quanta in 1 bit time, or 0xFFFF if this and the following fields have not been set
	uint16_t tseg1;					// now far into the period the sample point is, minimum 1, maximum period-2
	uint16_t jumpWidth;				// the (re)synchronisation jump width

	static constexpr uint16_t DefaultPeriod = 48;
	static constexpr uint16_t DefaultTseg1 = 26;
	static constexpr uint16_t DefaultJumpWidth = 8;

	bool IsValid() const
	{
		return period >= 24 && period <= 4800
			&& tseg1 != 0 && tseg1 <= period - 2;
	}

	void SetDefaults()
	{
		period = DefaultPeriod;
		tseg1 = DefaultTseg1;
		jumpWidth = DefaultJumpWidth;
	}
};

// This is read from the user area RAM, so all values default to all bits set
struct CanUserAreaData
{
	// Total 16 bytes
	uint8_t canIdV1NotSet : 1,					// set if canAddress does not contain the CAN address to use
			timingV1NotSet : 1,					// set if timing has not been set
			spare : 6;							// spare bits, must be set to 1 for future compatibility
	uint8_t canAddress;							// the CAN address of this board, or 0xFF if it has not been set
	CanTiming timing;
	uint16_t spare1, spare2, spare3, spare4;	// make up to 16 bytes for future expansion

	void SetCanAddress(CanAddress address)
	{
		if (address != CanId::MasterAddress && address < CanId::MaxNormalAddress)
		{
			canAddress = address;
			canIdV1NotSet = false;
		}
	}

	CanAddress GetCanAddress(CanAddress defaultAddress) const
	{
		const CanAddress ret = (canIdV1NotSet) ? defaultAddress : canAddress;
		return (canAddress == CanId::MasterAddress || canAddress > CanId::MaxNormalAddress)
			? CanId::FirmwareUpdateAddress
				: ret;
	}

	void SetTiming(const CanTiming& data)
	{
		if (data.IsValid())
		{
			timing = data;
			timingV1NotSet = false;
		}
	}

	void GetTiming(CanTiming& data) const
	{
		if (!timingV1NotSet && timing.IsValid())
		{
			data = timing;
		}
		else
		{
			data.SetDefaults();
		}
	}
};

static_assert(sizeof(CanUserAreaData) == 16);

#endif /* SRC_CANTIMINGDATA_H_ */
