/*
 * CanSettings.cpp
 *
 *  Created on: 30 Jan 2020
 *      Author: David
 */

#include "CanSettings.h"
#include "CanId.h"
#include <cstring>

uint16_t CanUserAreaData::GetChecksum() const noexcept
{
	const uint16_t *p = reinterpret_cast<const uint16_t*>(this);
	uint16_t csum = 0;
	for (unsigned int i = 0; i < sizeof(*this)/sizeof(uint16_t); ++i)
	{
		csum ^= *p++;
	}
	return csum;
}

bool CanUserAreaData::IsValid() const noexcept
{
	return GetChecksum() == magic;
}

void CanUserAreaData::UpdateChecksum() noexcept
{
	checksum ^= GetChecksum() ^ magic;
}

void CanUserAreaData::Clear() noexcept
{
	memset((void *)this, 0xFF, sizeof(*this));
}

void CanUserAreaData::SetCanAddress(CanAddress address) noexcept
{
	if (address != CanId::MasterAddress && address <= CanId::MaxNormalAddress)
	{
		if (!IsValid())
		{
			Clear();
		}
		canAddress = address;
		invertedCanAddress = ~canAddress;
		canIdV1NotSet = false;
		UpdateChecksum();
	}
}

bool CanUserAreaData::AddressValid() const noexcept
{
	return IsValid() && !canIdV1NotSet && canAddress == (uint8_t)(~invertedCanAddress) && canAddress != CanId::MasterAddress && canAddress <= CanId::MaxNormalAddress;
}

CanAddress CanUserAreaData::GetCanAddress(CanAddress defaultAddress) const noexcept
{
	return (AddressValid()) ? canAddress : defaultAddress;
}

void CanUserAreaData::SetTiming(const CanTiming& data) noexcept
{
	if (data.IsValid())
	{
		if (!IsValid())
		{
			Clear();
		}
		timing = data;
		timingV1NotSet = false;
		UpdateChecksum();
	}
}

void CanUserAreaData::GetTiming(CanTiming& data) const noexcept
{
	if (IsValid() && !timingV1NotSet && timing.IsValid())
	{
		data = timing;
	}
	else
	{
		data.SetDefaults();
	}
}

// End
