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

#if !defined(FDUC_H)
#define FDUC_H

#include "RingBuffer.h"
#include "IQSample.h"

#include <arm_math.h>

class IFDUC {
public:
    virtual ~IFDUC() = 0;

    virtual void process(CRingBuffer<IQSample<float32_t>>& in, CRingBuffer<IQSample<float32_t>>& out) = 0;

private:
};

class CFDUCDummy : public IFDUC {
public:
    CFDUCDummy();
    virtual ~CFDUCDummy();

    virtual void process(CRingBuffer<IQSample<float32_t>>& in, CRingBuffer<IQSample<float32_t>>& out);

private:
};

class CFDUC : public IFDUC {
public:
    CFDUC(
        unsigned int resampNum,
        unsigned int resampDen,
        int          rxIfNum,
        unsigned int rxIfDen,
        int          txIfNum,
        unsigned int txIfDen,
        // Approximate length of the filter in downconverted samples.
        // A higher value results in a narrower transition band
        // but higher CPU use.
        // Delay of the filter in downconverted samples
        // is approximately half of this value.
        unsigned int length,
        // Cutoff frequency as a fraction of Nyquist frequency
        // of downconverted sampler rate
        float32_t cutoff
    );

    virtual ~CFDUC();

    virtual void process(CRingBuffer<IQSample<float32_t>>& in, CRingBuffer<IQSample<float32_t>>& out);

private:
    // Numerator of sample rate ratio
    // This determines the interpolation factor for DDC
    // and decimation factor for DUC.
    unsigned int m_resampNum;
    // Denominator of sample rate ratio.
    // This determines the decimation factor for DDC
    // and interpolation factor for DUC.
    unsigned int m_resampDen;

    unsigned int m_branchlen;
    // Polyphase filter phase
    unsigned int m_p;
    // Index to m_in and m_out
    unsigned int m_i;
    // Index to m_duc_sine
    unsigned int m_duc_i;

    // Polyphase filter taps
    float32_t* m_taps;
    unsigned int m_tapsLen;

    // Input buffer for DUC resampler
    float32_t* m_outRe;
    float32_t* m_outIm;

    // Upconversion sine table
    float32_t* m_duc_sineI;
    float32_t* m_duc_sineQ;
    unsigned int m_duc_sineLen;

    // Scaling needed so that DUC has unity gain in passband
    float32_t    m_ducScaling;

    float32_t sinc(float32_t v) const;
    float32_t hann_window(unsigned int i, unsigned int length) const;
    float32_t windowed_sinc(unsigned int i, unsigned int length, float32_t cutoff) const;
    void      make_sine_table(float32_t* tableI, float32_t* tableQ, int freqNum, unsigned int freqDen) const;
};

#endif
