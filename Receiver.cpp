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

#include "Receiver.h"
#include "Thread.h"
#include "Log.h"

#include <cassert>

CReceiver::CReceiver(ReceiverDelegate* network, CSoapyDevice* device, CDMRTiming* burst_timer,
                   unsigned int num_active_channels, unsigned int num_pfb_channels, float sampleRate,
                   bool needs_timestamp, float symbol_deviation, float power_calibration) : 
m_running(true),
m_stopped(false),
m_timestamping(needs_timestamp),
m_network(network),
m_device(device),
m_burstTimer(burst_timer),
m_activeChannels(num_active_channels),
m_pfbChannels(num_pfb_channels),
m_powerCalibration(power_calibration),
m_symbolDeviation(symbol_deviation),
m_readTime(0LL)
{
    assert(m_activeChannels <= MAX_MMDVM_CHANNELS);
    assert(m_pfbChannels > 3U);
    assert(m_pfbChannels <= MAX_PFB_CHANNELS);

    for (unsigned int i = 0U;i < m_activeChannels;i++) {
        m_sampleBuf[i].reserve(SAMPLES_PER_SLOT);
        m_controlBuf[i].reserve(SAMPLES_PER_SLOT);
        m_RSSI[i] = 1024U;
    }

    unsigned int max_real_chan = m_pfbChannels / 2U - 1U;
    max_real_chan = std::min<unsigned int>(max_real_chan, 4U); // FIXME: flexible number of channels
    m_fillReal = std::min<unsigned int>(max_real_chan, m_activeChannels);

    //
    // Initialize channel receiving DSP (splitter, downsampler, demodulator)
    //
    m_firChannelAnalyzer = ::firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, m_pfbChannels, PFB_FILTER_DELAY, 70.0F);
    this->setRotateParams(DEFAULT_BASEBAND_SHIFT, sampleRate);

    ::memset(m_rreDownsampler, 0, sizeof(m_rreDownsampler));
    for (unsigned int i = 0U; i < sizeof(m_rreDownsampler) / sizeof(m_rreDownsampler[0]); i++) {
        m_rreDownsampler[i] = 0;
    }
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        m_rreDownsampler[i] = ::rresamp_crcf_create_kaiser(m_downsampleDecim, m_downsampleInterp, RESAMPLER_FILTER_DELAY, RESAMPLER_FRACTIONAL_BW, 70.0F);
    }

    for (unsigned int i = 0U; i < sizeof(m_FMdemod) / sizeof(m_FMdemod[0]); i++) {
        m_FMdemod[i] = 0;
    }
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
      m_FMdemod[i] = ::freqdem_create(FSK4_DEVIATION);
    }
}

CReceiver::~CReceiver()
{
    ::nco_crcf_destroy(m_ncoChannelRotator);
    ::firpfbch_crcf_destroy(m_firChannelAnalyzer);

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        ::rresamp_crcf_destroy(m_rreDownsampler[i]);
    }

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        ::freqdem_destroy(m_FMdemod[i]);
    }
}

void CReceiver::stop()
{
    m_running = false;

    LogDebug("Stopping Receiver Thread ...");
    wait();
}

bool CReceiver::stopped() const
{
    return m_stopped;
}

void CReceiver::entry()
{
    while (m_running) {
        std::complex<float> read_buffer[RX_SAMP_IN_SIZE];
        void *buffs[1] = {(void*)read_buffer};
        long long timeNs = 0LL;
    
        int flags = 0;
        int ret = m_device->getDevice()->readStream(m_device->getRxStream(), buffs, RX_INTERP_IN_SIZE * m_pfbChannels, flags, timeNs);
        if (ret <= 0) {
            ::LogError("Error reading samples from device: %s", SoapySDR_errToStr(ret));
            CThread::sleepMilli(3U);
            continue;
        }

        if ((unsigned int)ret != RX_INTERP_IN_SIZE * m_pfbChannels) {
            ::LogError("Underrun occurred while reading samples from device, only read %d samples!", ret);
            CThread::sleepMilli(3U);
            continue;
        }

        if (!m_timestamping || ((flags & SOAPY_SDR_HAS_TIME) != SOAPY_SDR_HAS_TIME))
            timeNs = m_readTime;

        std::complex<float> rotated[RX_SAMP_IN_SIZE] = {0.0F, 0.0F};
        this->rotateDown(read_buffer, RX_INTERP_IN_SIZE * m_pfbChannels, rotated);

        std::complex<float> channelized1[RX_INTERP_IN_SIZE][MAX_PFB_CHANNELS] = {{0.0F, 0.0F}};
        std::complex<float> channelized2[MAX_PFB_CHANNELS][RX_INTERP_IN_SIZE] = {{0.0F, 0.0F}};
        std::complex<float> rearranged[MAX_PFB_CHANNELS][RX_INTERP_IN_SIZE]   = {{0.0F, 0.0F}};

        for (unsigned int i = 0U; i < RX_INTERP_IN_SIZE; i++)
            this->channelize(&rotated[i*m_pfbChannels], &channelized1[i][0]);

        for (unsigned int i = 0U; i < m_pfbChannels; i++) {
            for (unsigned int j = 0U; j < RX_INTERP_IN_SIZE; j++)
                channelized2[i][j] = channelized1[j][i];
        }

        // First four usable channels are on the real side of the FFT, the rest in imag,
        // reversed order to minimize occupied BW for 250k sps
        // Channel 5 (6?) of the PFB wraps around to the imag side and is not usable
        for (unsigned int k = 0U; k < m_fillReal; k++) {
            for (unsigned int j = 0U; j < RX_INTERP_IN_SIZE; j++)
                rearranged[k][j] = channelized2[k][j];
        }

        for (unsigned int m = m_pfbChannels - 1U, p = m_fillReal; p < m_activeChannels; m--, p++) {
            for (unsigned int j = 0U; j < RX_INTERP_IN_SIZE; j++)
                rearranged[p][j] = channelized2[m][j];
        }

        m_burstTimer->lock();
        for (unsigned int j = 0U; j < m_activeChannels; j++) {
            m_burstTimer->setTimer(timeNs, j);

            float output_samples[RX_SAMP_OUT_SIZE] = { 0.0f };
            processSamples(j, rearranged[j], output_samples);

            for (unsigned int i = 0U; i < RX_SAMP_OUT_SIZE; i++) {
                uint8_t control = MARK_NONE;
                uint8_t slot_no = m_burstTimer->checkTime(j, i==0);

                if (slot_no == 1U)
                    control = MARK_SLOT1;
                else if (slot_no == 2U)
                    control = MARK_SLOT2;

                int32_t s = int32_t(32767.0F * output_samples[i] / m_symbolDeviation);
                s = (s > 32767) ? 32767 : s;
                s = (s < -32767) ? -32767 : s;

                int16_t sample = int16_t(s);
                m_sampleBuf[j].push_back(sample);
                m_controlBuf[j].push_back(control);
            }
        }
    
        m_burstTimer->unlock();

        // Simulate a timestamp
        if (!m_timestamping || ((flags & SOAPY_SDR_HAS_TIME) != SOAPY_SDR_HAS_TIME))
            m_readTime += (long long)ret * TIME_PER_SAMPLE * (long long)this->getDecim() /
                          (long long)this->getInterp() / (long long)m_pfbChannels;

        for (unsigned int j = 0U; j < m_activeChannels; j++) {
            uint32_t num_items = SAMPLES_PER_SLOT;
            if (m_sampleBuf[j].size() >= SAMPLES_PER_SLOT) {
                if (m_burstTimer->getInit(j)) {
#if 1
                    m_network->onRXSamples(m_controlBuf[j], m_sampleBuf[j], m_RSSI[j], j);
#else
                    unsigned int rssi = m_RSSI[j];
                    unsigned char reply[NETWORK_TX_PACKET_SIZE];
                    ::memcpy(reply, &num_items, sizeof(uint32_t));
                    ::memcpy(reply + sizeof(uint32_t), &rssi, sizeof(uint32_t));
                    ::memcpy(reply + 2U * sizeof(uint32_t), (unsigned char*)m_controlBuf[j].data(), num_items * sizeof(uint8_t));
                    ::memcpy(reply + 2U * sizeof(uint32_t) + num_items * sizeof(uint8_t), (unsigned char*)m_sampleBuf[j].data(), num_items*sizeof(int16_t));
                    m_network->write(reply, NETWORK_TX_PACKET_SIZE, j);
#endif
                }

                m_sampleBuf[j].erase(m_sampleBuf[j].begin(), m_sampleBuf[j].begin() + num_items);
                m_controlBuf[j].erase(m_controlBuf[j].begin(), m_controlBuf[j].begin() + num_items);
                m_sampleBuf[j].reserve(SAMPLES_PER_SLOT);
                m_controlBuf[j].reserve(SAMPLES_PER_SLOT);
                m_RSSI[j] = 1024U;
            }
        }
    }

    m_stopped = true;

    LogDebug("Receiver thread exit");
}

void CReceiver::processSamples(unsigned int channel, std::complex<float>* in_samples, float* output_samples)
{
    assert(channel < m_activeChannels);
    assert(in_samples != nullptr);
    assert(output_samples != nullptr);

    std::complex<float> resampled[RX_SAMP_OUT_SIZE] = {0.0F, 0.0F};
    this->downsample(channel, in_samples, RX_INTERP_IN_SIZE, resampled);

    float sum = 0.0F;
    for (unsigned int i = 0U; i < RX_SAMP_OUT_SIZE; i++)
        sum += std::norm(resampled[i]);

    float rms  = std::sqrt(((sum + 1.0E-20F) / float(RX_SAMP_OUT_SIZE)) / 2.0F);
    float pow  = (rms * rms) / 50.0F;
    float dbFS = 10.0F * std::log10(pow + 1.0E-20F);

    unsigned int rssi = (unsigned int)std::fabs(dbFS) + m_powerCalibration; // invert to positive, RSSI > 0 dBFs is unlikely
    if (rssi < m_RSSI[channel]) // keep max dB value since the buffer may overlap two timeslots, one active one inactive
        m_RSSI[channel] = rssi;

    this->demodulate(channel, resampled, RX_SAMP_OUT_SIZE, output_samples);
}

//
// For channel related methods
//
void CReceiver::setRotateParams(float rotation_hz, float sample_rate) {
    assert(sample_rate > 0.0F);

    nco_crcf nco = ::nco_crcf_create(LIQUID_VCO);
    ::nco_crcf_set_phase(nco, 0.0F);
    ::nco_crcf_set_frequency(nco, (2.0F * M_PI * rotation_hz / sample_rate));

    if (m_ncoChannelRotator)
        ::nco_crcf_destroy(m_ncoChannelRotator);
    
    m_ncoChannelRotator = nco;
}

void CReceiver::setDownsamplerParams(unsigned int interp, unsigned int decim, float bw) {
    assert(decim > 0U);

    m_downsampleInterp = interp;
    m_downsampleDecim = decim;

    for (unsigned int i = 0U; i < sizeof(m_rreDownsampler) / sizeof(m_rreDownsampler[0]); i++) {
        if (m_rreDownsampler[i]) {
            ::rresamp_crcf_destroy(m_rreDownsampler[i]);
            m_rreDownsampler[i] = 0;
        }
    }

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        m_rreDownsampler[i] = ::rresamp_crcf_create_kaiser(decim, interp, RESAMPLER_FILTER_DELAY, bw, 70.0F);
    }
}

void CReceiver::rotateDown(std::complex<float>* in_samples, unsigned int num_samples, std::complex<float>* out_samples) {
    assert(in_samples != nullptr);
	assert(out_samples != nullptr);

	::nco_crcf_mix_block_down(m_ncoChannelRotator, in_samples, out_samples, num_samples);
}

void CReceiver::channelize(std::complex<float>* in_samples, std::complex<float>* out_samples) {
    ::firpfbch_crcf_analyzer_execute(m_firChannelAnalyzer, in_samples, out_samples);
}

void CReceiver::downsample(unsigned int channel, std::complex<float>* in_samples,
                unsigned int num_samples, std::complex<float>* out_samples) {

    assert(in_samples != nullptr);
    assert(out_samples != nullptr);

    // TODO - Eliminate the heap-allocation 

    unsigned int interp = m_downsampleInterp;
    unsigned int decim  = m_downsampleDecim;

    // Interpolation and decimation are reversed when downsampling
    unsigned int p_in = num_samples / interp;

#if 1
    for (unsigned int i = 0U; i < p_in; i++) {
        ::rresamp_crcf_execute(m_rreDownsampler[channel], in_samples + (i * interp), out_samples + (i * decim));
    }
#else
    std::complex<float>* in_buf  = new std::complex<float>[interp];
    std::complex<float>* out_buf = new std::complex<float>[decim];

    for (unsigned int i = 0U; i < p_in; i++) {
        ::memcpy(in_buf, in_samples + (i * interp), interp * sizeof(std::complex<float>));
        ::rresamp_crcf_execute(m_rreDownsampler[channel], in_buf, out_buf);
        ::memcpy(out_samples + (i * decim), out_buf, decim * sizeof(std::complex<float>));
    }

    delete[] in_buf;
    delete[] out_buf;        
#endif
}

void CReceiver::demodulate(unsigned int channel, std::complex<float>* in_samples, const unsigned int num_samples, float* out_samples)
{
    assert(channel < m_activeChannels);
    assert(in_samples != nullptr);
    assert(out_samples != nullptr);

    ::freqdem_demodulate_block(m_FMdemod[channel], in_samples, num_samples, out_samples);
}
