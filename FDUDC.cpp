/*
 *   Copyright (C) 2025 by Jonathan Naylor G4KLX
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

#include "FDUDC.h"

#include "Complex.h"

#include <cassert>
#include <cmath>

#ifndef M_PIf32
#define M_PIf32 3.14159265358979323846f
#endif

const unsigned int FDUDC_VECLEN  = 4U;
const unsigned int FDUDC_VECSIZE = sizeof(float32_t) * FDUDC_VECLEN;

IFDUDC::~IFDUDC()
{
}

CFDUDCDummy::CFDUDCDummy(void (*process_sample)(float32_t& iValue, float32_t& qValue)) :
m_process_sample(process_sample)
{
}

CFDUDCDummy::~CFDUDCDummy()
{
}

void CFDUDCDummy::process(float32_t& iValue, float32_t& qValue)
{
    m_process_sample(iValue, qValue);
}

CFDUDC::CFDUDC(
    unsigned int resampNum,
    unsigned int resampDen,
    int          rxIfNum,
    unsigned int rxIfDen,
    int          txIfNum,
    unsigned int txIfDen,
    unsigned int length,
    float32_t cutoff,
    void (*process_sample)(float32_t& iValue, float32_t& qValue)
) :
    m_resampNum(resampNum),
    m_resampDen(resampDen),
    m_process_sample(process_sample),
    m_branchlen(0U),
    m_p(0U),
    m_i(0U),
    m_ddc_i(0U),
    m_duc_i(0U),
    m_iValue(0.0F),
    m_qValue(0.0F),
    m_taps(nullptr),
    m_tapsLen(0U),
    m_inRe(nullptr),
    m_inIm(nullptr),
    m_outRe(nullptr),
    m_outIm(nullptr),
    m_ddc_sineI(nullptr),
    m_ddc_sineQ(nullptr),
    m_ddc_sineLen(0U),
    m_duc_sineI(nullptr),
    m_duc_sineQ(nullptr),
    m_duc_sineLen(0U),
    m_ducScaling(0.0F)
{
    assert(process_sample != nullptr);

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
    m_branchlen = (m_branchlen + FDUDC_VECLEN - 1U) / FDUDC_VECLEN * FDUDC_VECLEN;

    // Total length of filter prototype:
    unsigned int totallen = m_branchlen * branches;

    m_taps = new float32_t[totallen];
    m_tapsLen = totallen;

    // Cutoff frequency in radians per sample
    float32_t sinc_cutoff = (cutoff * M_PIf32) / float32_t((resampDen));
    float32_t sum = 0.0F;

    for (unsigned int branch = 0U; branch < branches; branch++) {
        for (unsigned int i = 0U; i < m_branchlen; i++) {
            float32_t v = windowed_sinc(branches * i + branch, totallen, sinc_cutoff);

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

    // Additional scaling needed so that DUC has unity gain in passband
    m_ducScaling = float32_t(resampDen) / float32_t(resampNum);

    m_inRe = new float32_t[m_branchlen * 2U];
    m_inIm = new float32_t[m_branchlen * 2U];

    m_outRe = new float32_t[m_branchlen * 2U];
    m_outIm = new float32_t[m_branchlen * 2U];

    m_ddc_sineI = new float32_t[rxIfDen];
    m_ddc_sineQ = new float32_t[rxIfDen];
    m_ddc_sineLen = rxIfDen;

    m_duc_sineI = new float32_t[txIfDen];
    m_duc_sineQ = new float32_t[txIfDen];
    m_duc_sineLen = txIfDen;

    make_sine_table(m_ddc_sineI, m_ddc_sineQ, -rxIfNum, rxIfDen);
    make_sine_table(m_duc_sineI, m_duc_sineQ, txIfNum, txIfDen);
}

CFDUDC::~CFDUDC()
{
    delete[] m_taps;

    delete[] m_inRe;
    delete[] m_inIm;

    delete[] m_outRe;
    delete[] m_outIm;

    delete[] m_ddc_sineI;
    delete[] m_ddc_sineQ;

    delete[] m_duc_sineI;
    delete[] m_duc_sineQ;
}

void CFDUDC::process(float32_t& iValue, float32_t& qValue)
{
    float32_t real = 0.0F, imag = 0.0F;
    COMPLEX_MULT(real, imag, iValue, qValue, m_ddc_sineI[m_ddc_i], m_ddc_sineQ[m_ddc_i]);

    if (++m_ddc_i >= m_ddc_sineLen)
        m_ddc_i = 0U;

    m_inRe[m_i] = m_inRe[m_i + m_branchlen] = real;
    m_inIm[m_i] = m_inIm[m_i + m_branchlen] = imag;

    m_p += m_resampNum;
    while (m_p >= m_resampDen) {
        m_p -= m_resampDen;

        // Use DUC input from previous process_sample call.
        // This adds one sample of extra delay to DUC input.
        iValue = 0.0F;
        qValue = 0.0F;

        unsigned int iTap = m_p * m_branchlen;
        unsigned int iI   = m_i + 1U;

        for (unsigned int i = 0U; i < m_branchlen; i++) {
            const float32_t tap = m_taps[iTap];

            iValue += m_inRe[iI] * tap;
            qValue += m_inIm[iI] * tap;

            m_outRe[iI] += m_iValue * tap;
            m_outIm[iI] += m_qValue * tap;

            if (++iTap >= m_tapsLen)
                iTap = 0U;

            if (++iI >= (m_branchlen * 2U))
                iI = 0U;
        }

        m_process_sample(iValue, qValue);

        m_iValue = m_ducScaling * iValue;
        m_qValue = m_ducScaling * qValue;
    }

    COMPLEX_MULT(iValue, qValue,
        m_outRe[m_i] + m_outRe[m_i + m_branchlen],
        m_outIm[m_i] + m_outIm[m_i + m_branchlen],
        m_duc_sineI[m_duc_i],
        m_duc_sineQ[m_duc_i]);

    if (++m_duc_i >= m_duc_sineLen)
        m_duc_i = 0U;

    m_outRe[m_i] = m_outRe[m_i + m_branchlen] = 0.0F;
    m_outIm[m_i] = m_outIm[m_i + m_branchlen] = 0.0F;

    if (++m_i >= m_branchlen)
        m_i = 0U;
}

float32_t CFDUDC::sinc(float32_t v) const
{
    if (v == 0.0F)
        return 1.0F;
    else
        return ::sin(v) / v;
}

float32_t CFDUDC::hann_window(unsigned int i, unsigned int length) const
{
    return 0.5F - 0.5F * ::cos((0.5F + float32_t(i)) / float32_t(length) * (M_PIf32 * 2.0F));
}

float32_t CFDUDC::windowed_sinc(unsigned int i, unsigned int length, float32_t cutoff) const
{
    return sinc(cutoff * (float32_t(i) - 0.5F * float32_t(length))) * hann_window(i, length);
}

void CFDUDC::make_sine_table(float32_t* tableI, float32_t* tableQ, int freqNum, unsigned int freqDen) const
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
