/* 
 *   Copyright (C) 2018-2026 by Shawn Chain BG5HHP 
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
 * 
 *   @File: BiquadCascadeFilter 
 *   @Author: Shawn Chain 
 *   @Date: 2026-11-18 
 */
#if !defined(BIQUAD_CASCADE_FILTER)
#define	BIQUAD_CASCADE_FILTER

#include <cstdint>

#define BIQUAD_CASCADE_MAX_STAGES 12

struct BiquadCoeff {
    float b0, b1, b2;
    float a1, a2;
};

struct BiquadState {
    float x1, x2;
    float y1, y2;
};

class CBiquadCascadeFilter {
public:
    CBiquadCascadeFilter(const float* coeffArray, unsigned int stages, unsigned int coeffWidth, float gaindb = 0.f);

    void processSamples(float* samples, unsigned int nSamples);
    void processSamples(int16_t* samples, unsigned int nSamples);

    float filter(float sample);
    int16_t filter(int16_t sample);

    void reset();

private:
    BiquadCoeff m_coeffs[BIQUAD_CASCADE_MAX_STAGES];
    BiquadState m_states[BIQUAD_CASCADE_MAX_STAGES];
    unsigned int m_stages = 0;
};

#endif