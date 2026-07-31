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

#ifndef RECEIVER_H
#define RECEIVER_H

#include "SoapyDevice.h"
#include "Constants.h"
#include "DMRTiming.h"
#include "Thread.h"

#include <complex>
#include <cmath>
#include <liquid/liquid.h>

#include <string>
#include <cstdint>

class ReceiverDelegate {
public:
    virtual ~ReceiverDelegate(){};

    virtual void onRXSamples(const std::vector<uint8_t> &controlBuf, const std::vector<int16_t>& sampleBuf, unsigned int rssi, unsigned int channel) = 0;
};
class CReceiver : public CThread
{
public:
    CReceiver(ReceiverDelegate* network, CSoapyDevice* device, CDMRTiming* burst_timer, 
             unsigned int num_active_channels, unsigned int num_pfb_channels, float sampleRate,
             bool needs_timestamp, float symbol_deviation, float power_calibration);
    virtual ~CReceiver();

    virtual void entry();

    void stop();
    bool stopped() const;

    // From ReciverChannel
    void setRotateParams(float rotation_hz, float sample_rate);
    void setDownsamplerParams(unsigned int interp, unsigned int decim, float bw);

    void rotateDown(std::complex<float>* in_samples, unsigned int num_samples, std::complex<float>* out_samples);
    void channelize(std::complex<float>* in_samples, std::complex<float>* out_samples);

    void downsample(unsigned int channel, std::complex<float>* in_samples,
                    unsigned int num_samples, std::complex<float>* out_samples);

    void demodulate(unsigned int channel, std::complex<float>* in_samples, const unsigned int num_samples,float* out_samples);

    unsigned int getDecim() const { return m_downsampleDecim; };
    unsigned int getInterp() const { return m_downsampleInterp; };

private:
    void processSamples(unsigned int channel, std::complex<float>* in_samples, float* output_samples);

    bool m_running;
    bool m_stopped;
    bool m_timestamping;
    ReceiverDelegate* m_network;
    CSoapyDevice* m_device;
    CDMRTiming* m_burstTimer;
    unsigned int m_activeChannels;
    unsigned int m_pfbChannels;
    unsigned int m_fillReal;
    unsigned int m_powerCalibration;
    float m_symbolDeviation;
    long long m_readTime;
    std::vector<uint8_t> m_controlBuf[MAX_MMDVM_CHANNELS];
    std::vector<int16_t> m_sampleBuf[MAX_MMDVM_CHANNELS];
    unsigned int m_RSSI[MAX_MMDVM_CHANNELS];

    // receiver channels
    nco_crcf        m_ncoChannelRotator  = 0;        // no channel rotator
    firpfbch_crcf   m_firChannelAnalyzer = 0;       // fir channel splitter

    unsigned int    m_downsampleDecim   = RESAMPLER_DECIMATION;   // downsample resampler
    unsigned int    m_downsampleInterp  = RESAMPLER_INTERPOLATION;
    rresamp_crcf    m_rreDownsampler[MAX_MMDVM_CHANNELS];

    freqdem         m_FMdemod[MAX_MMDVM_CHANNELS];
};

#endif // RECEIVER_H
