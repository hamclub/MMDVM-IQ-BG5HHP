/*
 *   Copyright (C) 2023-2026 by Adrian Musceac YO8RZZ
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

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "SoapyDevice.h"
#include "Constants.h"
#include "DMRTiming.h"
#include "Thread.h"

#include <complex>
#include <cmath>
#include <liquid/liquid.h>

#include <string>
#include <vector>
#include <cstdint>

class TransmitterDelegate {
public:
    virtual ~TransmitterDelegate(){};

    virtual int getTXSamples(std::vector<uint8_t> &controlBuf, std::vector<float> &sampleBuf, float symbolDeviation, unsigned int channel) = 0;
};

class CTransmitter : public CThread
{
public:
    CTransmitter(TransmitterDelegate* network, CSoapyDevice* device, CDMRTiming* burst_timer,
                unsigned int num_active_channels, unsigned int num_pfb_channels, float sampleRate,
                bool needs_timestamp, float symbol_deviation, float dac_scaling);
    virtual ~CTransmitter();

    virtual void entry();

    bool isTXOn() { return m_tx; };
    void notifyTXDataUpdate(unsigned int channel);

    void stop();
    bool stopped() const;

    //
    // For channels
    //
    void setRotateParams(float rotation_hz, float sample_rate);
    void setUpsamplerParams(unsigned int interp, unsigned int decim, float bw);

    void rotateUp(std::complex<float>* in_samples, unsigned int num_samples, std::complex<float>* out_samples);

    void synthesize(std::complex<float>* in_samples, std::complex<float>* out_samples);

    void upsample(unsigned int channel, std::complex<float>* in_samples,
                    unsigned int num_samples, std::complex<float>* out_samples);

    void modulate(unsigned int channel, float* in_samples, const unsigned int num_samples, std::complex<float>* out_samples);

    unsigned int getDecim() const { return m_upsampleDecim; };

    unsigned int getInterp() const { return m_upsampleInterp; };

private:
    bool readNetwork();
    void processSamples(std::complex<float>* output_samples, bool* channel_idle);
    void setTx(bool* channelIdle);
    void nextSlot(unsigned int channel);

    bool m_running;
    bool m_stopped;
    bool m_timingInit;
    bool m_tx;
    bool m_timestamping;

    float m_DACScaling;
    float m_symbolDeviation;

    unsigned int m_activeChannels;
    unsigned int m_pfbChannels;
    unsigned int m_fillReal;

    int64_t      m_timingCorrection;
    TransmitterDelegate*     m_network;
    CSoapyDevice*      m_device;
    CDMRTiming*   m_burstTimer;

    std::complex<float>  m_txOutSampleBuffer[TX_SAMP_OUT_SIZE];
    std::complex<float>  m_channelizedSampleBuffer[TX_SAMP_OUT_SIZE];

    std::vector<uint8_t> m_controlBuf[MAX_MMDVM_CHANNELS];
    std::vector<float>   m_sampleBuf[MAX_MMDVM_CHANNELS];
    uint8_t              m_sn[MAX_MMDVM_CHANNELS];

    //
    // For channels
    nco_crcf        m_ncoChannelRotator = 0;
    firpfbch_crcf   m_firChannelSynthesizer = 0;

    unsigned int    m_upsampleDecim     = RESAMPLER_DECIMATION;
    unsigned int    m_upsampleInterp    = RESAMPLER_INTERPOLATION;
    rresamp_crcf    m_rreUpsampler[MAX_MMDVM_CHANNELS];

    freqmod         m_FMmod[MAX_MMDVM_CHANNELS];
};

#endif // TRANSMITTER_H
