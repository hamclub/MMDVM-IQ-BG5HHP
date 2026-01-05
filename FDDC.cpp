/*
 *   Copyright (C) 2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
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

#include "FDDC.h"

#include "Complex.h"

#include <cassert>
#include <cmath>

#ifndef M_PIf32
#define M_PIf32 3.14159265358979323846f
#endif

const unsigned int FDDC_VECLEN = 4U;
const unsigned int FDDC_VECSIZE = sizeof(float32_t) * FDDC_VECLEN;


CFDDC::CFDDC(
unsigned int resampNum,
unsigned int resampDen,
int          ifNum,
unsigned int ifDen,
unsigned int length,
float32_t    cutoff,
void (*callback)(const IQSample<float32_t>& sample)
) :
m_resampNum(resampNum),
m_resampDen(resampDen),
m_branchlen(0U),
m_p(0U),
m_i(0U),
m_sine_i(0U),
m_taps(nullptr),
m_tapsLen(0U),
m_inRe(nullptr),
m_inIm(nullptr),
m_sineI(nullptr),
m_sineQ(nullptr),
m_sineLen(0U),
m_callback(callback)
{
    assert(callback != nullptr);

    unsigned int approxlen = resampDen * length;

    // Number of filter branches:
    unsigned int branches = resampNum;

    // resampNum determines the number of polyphase branches.
    // Ensure the length is a multiple of that, so that each
    // branch has the same number of taps.
    // Additionally, the number of taps should be
    // a multiple of FDUDC_VECLEN, so round up to that.
    // Length of one branch:
    m_branchlen = (approxlen + branches - 1U) / branches;
    m_branchlen = (m_branchlen + FDDC_VECLEN - 1U) / FDDC_VECLEN * FDDC_VECLEN;

    // Total length of filter prototype:
    m_tapsLen = m_branchlen * branches;

    m_taps = new float32_t[m_tapsLen];

    // Cutoff frequency in radians per sample
    float32_t sinc_cutoff = (cutoff * M_PIf32) / float32_t((resampDen));
    float32_t sum = 0.0F;

    for (unsigned int branch = 0U; branch < branches; branch++) {
        for (unsigned int i = 0U; i < m_branchlen; i++) {
            float32_t v = windowed_sinc(branches * i + branch, m_tapsLen, sinc_cutoff);

            // Store taps of each branch contiguously
            m_taps[m_branchlen * branch + i] = v;

            sum += v;
        }
    }

    // Scale so that the sum over one filter branch becomes 1.
    // This gives the filter unity gain at DC.
    float32_t scaling = float32_t(branches) / sum;
    for (unsigned int i = 0U; i < m_tapsLen; i++)
        m_taps[i] *= scaling;

    m_inRe = new float32_t[m_branchlen * 2U];
    m_inIm = new float32_t[m_branchlen * 2U];

    m_sineI = new float32_t[ifDen];
    m_sineQ = new float32_t[ifDen];
    m_sineLen = ifDen;

    make_sine_table(m_sineI, m_sineQ, -ifNum, ifDen);
}

CFDDC::~CFDDC()
{
    delete[] m_taps;

    delete[] m_inRe;
    delete[] m_inIm;

    delete[] m_sineI;
    delete[] m_sineQ;
}

void CFDDC::process(const IQSample<float32_t>& sample)
{
    assert(m_callback != nullptr);

    float32_t real = 0.0F, imag = 0.0F;
    COMPLEX_MULT(real, imag, sample.iValue, sample.qValue, m_sineI[m_sine_i], m_sineQ[m_sine_i]);

    if (++m_sine_i >= m_sineLen)
        m_sine_i = 0U;

    m_inRe[m_i] = m_inRe[m_i + m_branchlen] = real;
    m_inIm[m_i] = m_inIm[m_i + m_branchlen] = imag;

    m_p += m_resampNum;
    while (m_p >= m_resampDen) {
        m_p -= m_resampDen;

        IQSample<float32_t> out;
        out.iValue  = 0.0F;
        out.qValue  = 0.0F;
        out.control = sample.control;

        unsigned int iTap = m_p * m_branchlen;
        unsigned int iI   = m_i + 1U;

        for (unsigned int i = 0U; i < m_branchlen; i++) {
            const float32_t tap = m_taps[iTap];

            out.iValue += m_inRe[iI] * tap;
            out.qValue += m_inIm[iI] * tap;

            if (++iTap >= m_tapsLen)
                iTap = 0U;

            if (++iI >= (m_branchlen * 2U))
                iI = 0U;
        }

        (*m_callback)(out);
    }

    if (++m_i >= m_branchlen)
        m_i = 0U;
}

float32_t CFDDC::sinc(float32_t v) const
{
    if (v == 0.0F)
        return 1.0F;
    else
        return ::sin(v) / v;
}

float32_t CFDDC::hann_window(unsigned int i, unsigned int length) const
{
    return 0.5F - 0.5F * ::cos((0.5F + float32_t(i)) / float32_t(length) * (M_PIf32 * 2.0F));
}

float32_t CFDDC::windowed_sinc(unsigned int i, unsigned int length, float32_t cutoff) const
{
    return sinc(cutoff * (float32_t(i) - 0.5F * float32_t(length))) * hann_window(i, length);
}

void CFDDC::make_sine_table(float32_t* tableI, float32_t* tableQ, int freqNum, unsigned int freqDen) const
{
    assert(tableI != nullptr);
    assert(tableQ != nullptr);

    // Frequency in radians per sample
    float32_t freq = float32_t(freqNum) / float32_t(freqDen) * (M_PIf32 * 2.0F);

    for (unsigned int i = 0U; i < freqDen; i++) {
        tableI[i] = ::cos(freq * float32_t(i));
        tableQ[i] = ::sin(freq * float32_t(i));
    }
}
