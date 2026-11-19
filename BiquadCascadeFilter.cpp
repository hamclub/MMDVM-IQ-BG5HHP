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

#include "BiquadCascadeFilter.h"

#include <cstring>
#include <cassert>
#include <cmath>

/**
 * @brief Construct a new CBiquadCascadeFilter::CBiquadCascadeFilter object
 * 
 * @param coeffArray 
 * @param stages 
 * @param coeffWidth 5 means cmsis-dsp format, 6 means numpy format
 * @param gaindb additional gain applied to the filter
 */
CBiquadCascadeFilter::CBiquadCascadeFilter(const float* coeffArray, unsigned int stages, unsigned int coeffWidth, float gaindb) {
    if (stages > BIQUAD_CASCADE_MAX_STAGES)
        stages = BIQUAD_CASCADE_MAX_STAGES;

    ::memset(m_coeffs, 0, sizeof(m_coeffs));
    ::memset(m_states, 0, sizeof(m_states));
    m_stages = stages;

    const float* coeff = coeffArray;

    for (unsigned int i = 0; i < stages; i++) {
        if (coeffWidth == 6 /* NumPy coeffs format */) {
            m_coeffs[i].b0 = coeff[0];
            m_coeffs[i].b1 = coeff[1];
            m_coeffs[i].b2 = coeff[2];
            // m_coeffs[i].a0 = fa[3];
            m_coeffs[i].a1 = coeff[4];
            m_coeffs[i].a2 = coeff[5];
            coeff += 6;
        } else /* CMSIS-DSP format */{
            m_coeffs[i].b0 = coeff[0];
            m_coeffs[i].b1 = coeff[1];
            m_coeffs[i].b2 = coeff[2];
            m_coeffs[i].a1 = -coeff[3];
            m_coeffs[i].a2 = -coeff[4];
            coeff += 5;
        }
    }

    float gainx = ::powf(10.0F, gaindb / 20.f);
    if (gainx > 0.f) {
        m_coeffs[0].b0 *= gainx;
        m_coeffs[0].b1 *= gainx;
        m_coeffs[0].b2 *= gainx;
    }
}

void CBiquadCascadeFilter::processSamples(float* samples, unsigned int nSamples) {
    for (unsigned int i = 0; i < nSamples; i++) {
        samples[i] = this->filter(samples[i]);
    }
}

void CBiquadCascadeFilter::processSamples(int16_t* samples, unsigned int nSamples) {
    for (unsigned int i = 0; i < nSamples; i++) {
        samples[i] = this->filter(samples[i]);
    }
}

float CBiquadCascadeFilter::filter(float sample) {
    assert(m_stages <= sizeof(m_states) / sizeof(m_states[0]));

    float input = sample;
    float output = 0.f;

    float b0, b1, b2, a1, a2;
    float x1,y1;

    for (unsigned int i = 0; i < m_stages; i++) {
        b0 = m_coeffs[i].b0;
        b1 = m_coeffs[i].b1;
        b2 = m_coeffs[i].b2;
        a1 = m_coeffs[i].a1;
        a2 = m_coeffs[i].a2;

        x1 = m_states[i].x1;
        y1 = m_states[i].y1;

        output = b0 * input + b1 * x1 + b2 * m_states[i].x2 - a1 * y1 - a2 * m_states[i].y2;

        m_states[i].x2 = x1;
        m_states[i].y2 = y1;
        m_states[i].x1 = input;
        m_states[i].y1 = output;

        input = output;
    }

    return output;
}

int16_t CBiquadCascadeFilter::filter(int16_t sample) {
    float fOutput = this->filter((float)sample / 32768.f);

    int32_t iOutput = (int32_t)(fOutput * 32768.f + 0.5f);
    if (iOutput > 32767)
        iOutput = 32767;
    else if (iOutput < -32768)
        iOutput = -32768;

    return (int16_t)iOutput;
}

void CBiquadCascadeFilter::reset() {
    assert(m_stages <= sizeof(m_states) / sizeof(m_states[0]));

    for (unsigned int i = 0; i < m_stages; i++) {
        m_states[i].x1 = 0.f;
        m_states[i].x2 = 0.f;
        m_states[i].y1 = 0.f;
        m_states[i].y2 = 0.f;
    }
}