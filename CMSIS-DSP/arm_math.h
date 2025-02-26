/*
 *   Copyright (C) 2024 by Jonathan Naylor G4KLX
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

#if !defined(ARM_MATH_H)
#define  ARM_MATH_H

#include <cstdint>

typedef int8_t  q7_t;
typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int64_t q63_t;

typedef float float32_t;

struct arm_fir_instance_q15 {
	uint16_t     numTaps;
	q15_t*       pState;
	const q15_t* pCoeffs;
};

struct arm_fir_interpolate_instance_q15 {
	uint8_t      L;
	uint16_t     phaseLength;
	const q15_t* pCoeffs;
	q15_t*       pState;
};

struct arm_fir_instance_f32 {
	uint16_t         numTaps;
	float32_t*       pState;
	const float32_t* pCoeffs;
};

struct arm_biquad_casd_df1_inst_q31 {
	uint32_t     numStages;
	q31_t*       pState;
	const q31_t* pCoeffs;
	uint8_t      postShift;
};

void arm_q15_to_q31(const q15_t* pSrc, q31_t* pDst, uint32_t blockSize);

q15_t arm_sin_q15(q15_t x);

q15_t arm_cos_q15(q15_t x);

q31_t arm_sin_q31(q31_t x);

q31_t arm_cos_q31(q31_t x);

q31_t arm_atan_q31(q31_t y, q31_t x);

void arm_sqrt_q31(q31_t in, q31_t* out);

int32_t __SSAT(int32_t val, uint32_t sat);

void arm_abs_q31(const q31_t* pSrc, q31_t* pDst, uint32_t blockSize);

void arm_fir_fast_q15(const arm_fir_instance_q15* S, const q15_t* pSrc, q15_t* pDst, uint32_t blockSize);

void arm_fir_interpolate_q15(const arm_fir_interpolate_instance_q15* S, const q15_t* pSrc, q15_t* pDst, uint32_t blockSize);

void arm_fir_f32(const arm_fir_instance_f32* S, const float32_t* pSrc, float32_t* pDst, uint32_t blockSize);

void arm_biquad_cascade_df1_q31(const arm_biquad_casd_df1_inst_q31* S, const q31_t* pSrc, q31_t* pDst, uint32_t blockSize);

#endif
