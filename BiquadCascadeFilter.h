/*
 *   Copyright (C) 2015-2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2026 by Shawn Chain - BG5HHP
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
#if !defined(BIQUAD_CASCADE_FILTER)
#define	BIQUAD_CASCADE_FILTER

#include <cstdint>

#define BIQUAD_CASCADE_MAX_STAGES 10

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
    CBiquadCascadeFilter(const float* coeffArray, unsigned int stages, unsigned int coeffWidth = 6, float gaindb = 0.f);
    float process(float sample);
    int16_t process(int16_t sample);
    void reset();

private:
    BiquadCoeff m_coeffs[BIQUAD_CASCADE_MAX_STAGES];
    BiquadState m_states[BIQUAD_CASCADE_MAX_STAGES];
    unsigned int m_stages = 0;
};

#endif