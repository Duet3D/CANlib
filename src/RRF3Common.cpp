/*
 * RRF3Common.cpp
 *
 *  Created on: 18 Oct 2021
 *      Author: David
 */

#include "RRF3Common.h"
#include <General/Bitmap.h>

// Append the driver status as text. 'severity' meaning is 0 for all, 1 for warnings/errors, 2 for errors only.
void StandardDriverStatus::AppendText(const StringRef& str, unsigned int severity) const noexcept
{
	// Report any errors
	const uint32_t relevantBits = (severity == 2) ? all & ErrorMask
									: (severity == 1) ? all & (WarningMask | ErrorMask)
										: all;
	Bitmap<uint16_t> errors(relevantBits);
	errors.Iterate([&str](unsigned int bitNum, unsigned int index)->void
					{
						if (index != 0)
						{
							str.cat(", ");
						}
						str.cat(BitMeanings[bitNum]);
					}
				);
}

// End
