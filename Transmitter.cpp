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

#if defined(USE_SOAPY_MULTI)

#include "Transmitter.h"
#include "Thread.h"
#include "Log.h"

#include <cassert>

CTransmitter::CTransmitter(TransmitterDelegate* network, CSoapyDevice* device, CDMRTiming* burst_timer, 
                         unsigned int num_active_channels, unsigned int num_pfb_channels, float sampleRate,
                         bool needs_timestamp, float symbol_deviation, float dac_scaling) : 
m_running(true),
m_stopped(false),
m_timingInit(false),
m_tx(false),
m_timestamping(needs_timestamp),
m_DACScaling(dac_scaling),
m_symbolDeviation(symbol_deviation),
m_activeChannels(num_active_channels),
m_pfbChannels(num_pfb_channels),
m_timingCorrection(0LL),
m_network(network),
m_device(device),
m_burstTimer(burst_timer)
{
    assert(m_activeChannels <= MAX_MMDVM_CHANNELS);
    assert(m_pfbChannels > 3U);
    assert(m_pfbChannels <= MAX_PFB_CHANNELS);

    for (unsigned int i = 0U; i < m_activeChannels; i++)
        m_sn[i] = 1;

    unsigned int max_chan_real = m_pfbChannels / 2U - 1U;
    max_chan_real = std::min<unsigned int>(max_chan_real, 4U); // FIXME: flexible number of channels
    m_fillReal    = std::min<unsigned int>(max_chan_real, m_activeChannels);

    //
    // Initialize channel transmitting DSP (synthesizer, upsampler, modulator)
    //
    m_firChannelSynthesizer = ::firpfbch_crcf_create_kaiser(LIQUID_SYNTHESIZER, m_pfbChannels, PFB_FILTER_DELAY, 70.0F);
    this->setRotateParams(DEFAULT_BASEBAND_SHIFT, sampleRate);

    for (unsigned int i = 0U; i < sizeof(m_rreUpsampler) / sizeof(m_rreUpsampler[0]); i++) {
        m_rreUpsampler[i] = 0;
    }
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        m_rreUpsampler[i] = ::rresamp_crcf_create_kaiser(m_upsampleInterp, m_upsampleDecim, RESAMPLER_FILTER_DELAY, RESAMPLER_FRACTIONAL_BW, 70.0F);
    }

    for (unsigned int i = 0U; i < sizeof(m_FMmod) / sizeof(m_FMmod[0]); i++) {
        m_FMmod[i] = 0;
    }
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
      m_FMmod[i] = ::freqmod_create(FSK4_DEVIATION);
    }
}

CTransmitter::~CTransmitter()
{
    ::nco_crcf_destroy(m_ncoChannelRotator);
    ::firpfbch_crcf_destroy(m_firChannelSynthesizer);

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        ::rresamp_crcf_destroy(m_rreUpsampler[i]);
    }

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        ::freqmod_destroy(m_FMmod[i]);
    }
}

void CTransmitter::stop()
{
    m_running = false;

    LogDebug("Stopping Transmitter Thread ...");

    wait();
}

bool CTransmitter::stopped() const
{
    return m_stopped;
}

void CTransmitter::nextSlot(unsigned int channel)
{
    if (m_sn[channel] == 2U)
        m_sn[channel] = 1U;
    else
        m_sn[channel] = 2U;
}

void CTransmitter::notifyTXDataUpdate(unsigned int channel) {
    // trigger fetch tx samples
    m_network->getTXSamples(m_controlBuf[channel], m_sampleBuf[channel], m_symbolDeviation, channel);
}

bool CTransmitter::readNetwork()
{
    bool hasData = false;
    for (unsigned int j = 0U; j < m_activeChannels; j++) {
#if 1
        int ret = m_network->getTXSamples(m_controlBuf[j], m_sampleBuf[j], m_symbolDeviation, j);
        if (ret > 0)
            hasData = true;
#else
        unsigned char reply_message[NETWORK_RX_PACKET_SIZE]; // 720 * 3 + 4
        int ret = m_network->read(reply_message, NETWORK_RX_PACKET_SIZE, j);

        if ((unsigned int)ret != NETWORK_RX_PACKET_SIZE)
            continue;

        // the code used to work with variable packet sizes, but it's no longer needed
        uint32_t num_items = 0U;
        ::memcpy(&num_items, reply_message, sizeof(uint32_t));

        if (num_items == SAMPLES_PER_SLOT) {
            uint8_t control[SAMPLES_PER_SLOT];
            int16_t data[SAMPLES_PER_SLOT];
            ::memcpy(&control, reply_message + sizeof(uint32_t), num_items * sizeof(uint8_t));
      
            ::memcpy(&data, reply_message + sizeof(uint32_t) + num_items * sizeof(uint8_t), num_items * sizeof(int16_t));

            for (unsigned int i = 0U; i < num_items; i++) {
                m_controlBuf[j].push_back(control[i]);
                m_sampleBuf[j].push_back((float(data[i]) * m_symbolDeviation)/32767.0f);
            }
        }
#endif
    }

    return hasData;
}

void CTransmitter::entry()
{
    while(m_running) {
        long long timeNs = 0LL;

        if (!m_timingInit) {
            m_burstTimer->lock();
            bool has_time = true;

            for (unsigned int i = 0U; i < m_activeChannels; i++) {
                if (!m_burstTimer->getInit(i)) {
                    has_time = false;
                    break;
                }
            }

            m_burstTimer->unlock();

            if (has_time) {
                m_timingInit = true;

                // To avoid race condition with LMS7002M calibration routine, wait 120 msec after init to set gain to minimum
                m_device->setTx(false);
            }
        }

        if (!m_timingInit) {
            CThread::sleepMilli(2U);
            continue;
        }

        readNetwork();

        if (m_timingCorrection > 0) {
            CThread::sleepNano(m_timingCorrection);
            m_timingCorrection = 0;
        }

        bool channelIdle[MAX_MMDVM_CHANNELS];
        bool chanTiming[MAX_MMDVM_CHANNELS];
        for (unsigned int i = 0U; i < MAX_MMDVM_CHANNELS; i++) {
            channelIdle[i] = false;
            chanTiming[i]  = false;
        }

        m_burstTimer->lock();
        for (unsigned int i = 0U; i < m_activeChannels; i++) {
            long long time = 0LL;
            if (m_sampleBuf[i].size() < 1) {
                channelIdle[i] = true;
                m_sampleBuf[i].insert(m_sampleBuf[i].begin(), SAMPLES_PER_SLOT, 0.0F);
                m_controlBuf[i].insert(m_controlBuf[i].begin(), SAMPLES_PER_SLOT, MARK_NONE);
                time = m_burstTimer->allocateSlot(m_sn[i], m_timingCorrection, i) - (MMDVM_MARK_POSITION * TIME_PER_SAMPLE);

                if (timeNs == 0LL)
                    timeNs = time;

                chanTiming[i] = true;
                nextSlot(i);
            } else {
                for (unsigned int j = 0U; j < m_controlBuf[i].size(); j++) {
                    uint8_t control = m_controlBuf[i].at(j);
                    if (control == MARK_SLOT1) {
                        chanTiming[i] = true;
                        m_sn[i] = 1U;
                        time = m_burstTimer->allocateSlot(1U, m_timingCorrection, i) - (j * TIME_PER_SAMPLE);
                        if (timeNs == 0LL)
                            timeNs = time;
                    } else if (control == MARK_SLOT2) {
                        chanTiming[i] = true;
                        m_sn[i] = 2U;
                        time = m_burstTimer->allocateSlot(2U, m_timingCorrection, i) - (j * TIME_PER_SAMPLE);
                        if (timeNs == 0LL)
                            timeNs = time;
                    }
                }
            }

            if (!chanTiming[i]) {
                time = m_burstTimer->allocateSlot(m_sn[i], m_timingCorrection, i) - (MMDVM_MARK_POSITION * TIME_PER_SAMPLE);
                if (timeNs == 0LL)
                    timeNs = time;
                chanTiming[i] = true;
                nextSlot(i);
            }
        }

        m_burstTimer->unlock();

        setTx(channelIdle);

        // std::complex<float> output_samples[TX_SAMP_OUT_SIZE] = {0.0F, 0.0F};
        // ::memset(m_txOutSampleBuffer, 0, sizeof(m_txOutSampleBuffer));
        std::fill_n(m_txOutSampleBuffer, sizeof(m_txOutSampleBuffer) / sizeof(m_txOutSampleBuffer[0]), 0);
        std::complex<float> *output_samples = m_txOutSampleBuffer;
        processSamples(output_samples, channelIdle);

        void* buffs[1] = {(void*)output_samples};

        int flags = 0;
        if (m_timestamping && (timeNs > 0LL))
            flags |= SOAPY_SDR_HAS_TIME;

        int ret = m_device->getDevice()->writeStream(m_device->getTxStream(), buffs, TX_INTERP_OUT_SIZE * m_pfbChannels, flags, timeNs);
        if (ret <= 0) {
            ::LogError("Error writing samples to device: %s", SoapySDR_errToStr(ret));
        } else if ((unsigned int)ret != (TX_INTERP_OUT_SIZE * m_pfbChannels)) {
            ::LogError("TX overrun occured, only wrote %d samples!", ret);
        }
    }

    m_stopped = true;
    LogMessage("Transmitter thread exit");
}

void CTransmitter::processSamples(std::complex<float>* output_samples, bool* channel_idle)
{
    assert(output_samples != nullptr);
    assert(channel_idle != nullptr);

    std::complex<float> channelizer_samples[MAX_MMDVM_CHANNELS][TX_INTERP_OUT_SIZE] = {{0.0F, 0.0F}};
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        if (m_sampleBuf[i].size() >= SAMPLES_PER_SLOT) {
            std::complex<float> freq_modulated[SAMPLES_PER_SLOT] = {0.0F, 0.0F};
            this->modulate(i, m_sampleBuf[i].data(), SAMPLES_PER_SLOT, freq_modulated);

            if (channel_idle[i]) {
                for (unsigned int j = 0U; j < SAMPLES_PER_SLOT; j++)
                    freq_modulated[j] = {0.0F, 0.0F}; // zero it but keep next phase step in freqmod
            }

#if 1
            this->upsample(i, freq_modulated, SAMPLES_PER_SLOT, &channelizer_samples[i][0]);
#else
            std::complex<float> resampled[TX_INTERP_OUT_SIZE] = {0.0F, 0.0F};
            this->upsample(i, freq_modulated, SAMPLES_PER_SLOT, resampled);
            ::memcpy(&channelizer_samples[i][0], resampled, TX_INTERP_OUT_SIZE * sizeof(std::complex<float>));
#endif
            m_sampleBuf[i].erase(m_sampleBuf[i].begin(), m_sampleBuf[i].begin() + SAMPLES_PER_SLOT);
            m_controlBuf[i].erase(m_controlBuf[i].begin(), m_controlBuf[i].begin() + SAMPLES_PER_SLOT);
        }
    }
  
    // std::complex<float> channelized[TX_SAMP_OUT_SIZE] = {0.0F, 0.0F};
    // ::memset(m_channelizedSampleBuffer, 0, sizeof(m_channelizedSampleBuffer));
    std::fill_n(m_channelizedSampleBuffer, sizeof(m_channelizedSampleBuffer) / sizeof(m_channelizedSampleBuffer[0]), 0);
    *m_channelizedSampleBuffer = {0.0F, 0.0F};
    std::complex<float> *channelized = m_channelizedSampleBuffer;
    std::complex<float> channels[MAX_PFB_CHANNELS] = {0.0F, 0.0F};

    for (unsigned int i = 0U; i < TX_INTERP_OUT_SIZE; i++) {
        for (unsigned int k = 0U; k < m_fillReal; k++)
            channels[k] = channelizer_samples[k][i] * m_DACScaling / float(m_activeChannels);

        for (unsigned int m = m_pfbChannels - 1U, p = m_fillReal; p < m_activeChannels; m--, p++)
            channels[m] = channelizer_samples[p][i] * m_DACScaling / float(m_activeChannels);

        this->synthesize(channels, &channelized[i * m_pfbChannels]);
    }

    this->rotateUp(channelized, TX_INTERP_OUT_SIZE * m_pfbChannels, output_samples);
}

void CTransmitter::setTx(bool* channelIdle)
{
    assert(channelIdle != nullptr);

    bool active = false;
    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        if (!channelIdle[i]) {
            active = true;
            break;
        }
    }

    if (!m_tx && active) {
        m_tx = true;
        m_device->setTx(true);
        ::LogMessage("Transmitter TX on");
    } else if (m_tx && !active) {
        m_tx = false;
        m_device->setTx(false);
        ::LogMessage("Transmitter TX off");
    }
}

//
// Channel transmitting DSP
//
void CTransmitter::setRotateParams(float rotation_hz, float sample_rate) {
    assert(sample_rate > 0.0F);

    nco_crcf nco = ::nco_crcf_create(LIQUID_VCO);
    ::nco_crcf_set_phase(nco, 0.0F);
    ::nco_crcf_set_frequency(nco, (2.0F * M_PI * rotation_hz / sample_rate));

    if (m_ncoChannelRotator)
        ::nco_crcf_destroy(m_ncoChannelRotator);
    
    m_ncoChannelRotator = nco;
}

void CTransmitter::setUpsamplerParams(unsigned int interp, unsigned int decim, float bw) {
    assert(decim > 0U);

    m_upsampleInterp = interp;
    m_upsampleDecim = decim;

    for (unsigned int i = 0U; i < sizeof(m_rreUpsampler) / sizeof(m_rreUpsampler[0]); i++) {
        if (m_rreUpsampler[i]) {
            ::rresamp_crcf_destroy(m_rreUpsampler[i]);
        }
    }

    for (unsigned int i = 0U; i < m_activeChannels; i++) {
        m_rreUpsampler[i] = ::rresamp_crcf_create_kaiser(interp, decim, RESAMPLER_FILTER_DELAY, bw, 70.0F);
    }
}

void CTransmitter::rotateUp(std::complex<float>* in_samples, unsigned int num_samples, std::complex<float>* out_samples) {
    assert(in_samples != nullptr);
	assert(out_samples != nullptr);

	::nco_crcf_mix_block_up(m_ncoChannelRotator, in_samples, out_samples, num_samples);
}

void CTransmitter::synthesize(std::complex<float>* in_samples, std::complex<float>* out_samples) {
    ::firpfbch_crcf_synthesizer_execute(m_firChannelSynthesizer, in_samples, out_samples);
}

void CTransmitter::upsample(unsigned int channel, std::complex<float>* in_samples, unsigned int num_samples, std::complex<float>* out_samples)
{
    assert(in_samples != nullptr);
    assert(out_samples != nullptr);

    // TODO - Eliminate the heap-allocation

    unsigned int decim = m_upsampleDecim;
    unsigned int interp = m_upsampleInterp;

    unsigned int p_in = num_samples / decim;
#if 1   // zero copy
    for (unsigned int i = 0U; i < p_in; i++) {
        ::rresamp_crcf_execute(m_rreUpsampler[channel], in_samples + (i * decim), out_samples + (i * interp));
    }
#else
    std::complex<float>* in_buf  = new std::complex<float>[decim];
    std::complex<float>* out_buf = new std::complex<float>[interp];

    for (unsigned int i = 0U; i < p_in; i++) {
        ::memcpy(in_buf, in_samples + (i * decim), decim * sizeof(std::complex<float>));
        ::rresamp_crcf_execute(m_rreUpsampler[channel], in_buf, out_buf);
        ::memcpy(out_samples + (i * interp), out_buf, interp * sizeof(std::complex<float>));
    }

    delete[] in_buf;
    delete[] out_buf;
#endif
}

void CTransmitter::modulate(unsigned int channel, float* in_samples, const unsigned int num_samples, std::complex<float>* out_samples)
{
    assert(channel < m_activeChannels);
    assert(in_samples != nullptr);
    assert(out_samples != nullptr);

    ::freqmod_modulate_block(m_FMmod[channel], in_samples, num_samples, out_samples);
}

#endif