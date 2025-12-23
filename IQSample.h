/*
 *   Copyright (C) 2025 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(IQSAMPLE_H)
#define  IQSAMPLE_H

#include <cstdint>

template <typename TDATATYPE>
class IQSample {
public:
	TDATATYPE iValue;
	TDATATYPE qValue;
	uint8_t   control;

	IQSample() :
	iValue(),
	qValue(),
	control(0U)
	{
	}

	IQSample(const IQSample& sample) :
	iValue(sample.iValue),
	qValue(sample.qValue),
	control(sample.control)
	{
	}

	IQSample& operator=(const IQSample& sample)
	{
		if (&sample != this) {
			iValue  = sample.iValue;
			qValue  = sample.qValue;
			control = sample.control;
		}

		return *this;
	}
};

#endif
