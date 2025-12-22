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

#include "arm_math.h"


class IQSampleU16 {
public:
	uint16_t iValue;
	uint16_t qValue;
	uint8_t  control;

	IQSampleU16() :
	iValue(0U),
	qValue(0U),
	control(0U)
	{
	}

	IQSampleU16(const IQSampleU16& sample) :
	iValue(sample.iValue),
	qValue(sample.qValue),
	control(sample.control)
	{
	}

	IQSampleU16& operator=(const IQSampleU16& sample)
	{
		if (&sample != this) {
			iValue  = sample.iValue;
			qValue  = sample.qValue;
			control = sample.control;
		}

		return *this;
	}
};

class IQSampleF32 {
public:
	float32_t iValue;
	float32_t qValue;
	uint8_t   control;

	IQSampleF32() :
	iValue(0.0F),
	qValue(0.0F),
	control(0U)
	{
	}

	IQSampleF32(const IQSampleF32& sample) :
	iValue(sample.iValue),
	qValue(sample.qValue),
	control(sample.control)
	{
	}

	IQSampleF32& operator=(const IQSampleF32& sample)
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
