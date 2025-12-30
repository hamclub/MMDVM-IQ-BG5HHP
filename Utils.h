/*
 *   Copyright (C) 2015,2016,2020 by Jonathan Naylor G4KLX
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

#if !defined(UTILS_H)
#define  UTILS_H

#include <Arduino.h>

#include "arm_math.h"

static inline q15_t FLOAT32_TO_Q15(float32_t in)
{
    if (in > 1.0F)
        return INT16_MAX;
    else if (in < -1.0f)
        return INT16_MIN;
    else
        return q15_t((float32_t(INT16_MAX) * in) + 0.5F);
}

static inline int16_t FLOAT32_TO_INT16(float32_t in)
{
    if (in > 1.0F)
        return INT16_MAX;
    else if (in < -1.0f)
        return INT16_MIN;
    else
        return q15_t((float32_t(INT16_MAX) * in) + 0.5F);
}

static inline uint16_t FLOAT32_TO_UINT16(float32_t in)
{
    if (in > 1.0F)
        return UINT16_MAX;
    else if (in < -1.0f)
        return 0U;
    else
        return uint16_t(float32_t(UINT16_MAX) * ((in + 1.0F) / 2.0F));
}

static inline float32_t Q15_TO_FLOAT32(q15_t in)
{
    return float32_t(in) / float32_t(INT16_MAX);
}

static inline float32_t INT16_TO_FLOAT32(int16_t in)
{
    return float32_t(in) / float32_t(INT16_MAX);
}

static inline float32_t UINT16_TO_FLOAT32(uint16_t in)
{
    return ((float32_t(in) / float32_t(UINT16_MAX)) * 2.0F) - 1.0F;
}

uint8_t countBits8(uint8_t bits);

uint8_t countBits16(uint16_t bits);

uint8_t countBits32(uint32_t bits);

uint8_t countBits64(uint64_t bits);

#endif

