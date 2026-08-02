/*
 *   Copyright (C) 2026 by Shawn Chain BG5HHP
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

#include "SDRSoapyMulti.h"

#include "Conf.h"

#include "Modem.h"

#include <cassert>

#define LOG_SAMPLE_INTERVAL(str, nSamples, channel) \
    {                                                       \
        static unsigned long long t = 0;                    \
        static unsigned int c = 0;                          \
        static const unsigned int count = 100;              \
        if (t == 0)                                         \
            t = CTimer::getCurrentClockMillis();            \
                                                            \
        if (++c >= count) {                                 \
            unsigned long long e = CTimer::getCurrentClockMillis() - t; \
            LogDebug(str " %u, ch %u, interval: %.2f ms", nSamples, channel, (float)e / (float)count); \
            t = 0;                                          \
            c = 0;                                          \
        }                                                   \
    }


CSDRSoapyMulti::CSDRSoapyMulti(CConf* conf) : 
m_conf(conf),
m_rxMutex(),
m_txMutex()
{
    assert(conf);

    // Determin device sample rate
    std::string modemType = conf->getModemType();
    std::string modemURI  = conf->getModemURI();
    // LogDebug("SDRSoapyMulti Device: %s, %s", modemType.c_str(), modemURI.c_str());

    if (modemType.compare("sx") == 0 || modemType.compare("mucell") == 0) {
        m_sampleRate = 150000UL;  // 150k SPS, 6 x 25kHz, max pfb = 6
    } else {
        m_sampleRate = 250000UL;  // 250k SPS, 10 x 25Khz, max pfb = 10
    }
    // LogDebug("SDRSoapyMulti SampleRate: %u", m_sampleRate);

    // Setup multi channels
    unsigned int activeChannels = conf->getActiveChannels();
    this->setChannels(10, activeChannels);

    for (unsigned int i = 0; i < MAX_MMDVM_CHANNELS; i++) {
        m_rxIQBuffer[i] = nullptr;
        m_txIQBuffer[i] = nullptr;

        m_modems[i] = nullptr;
        m_txTimeout[i] = nullptr;
    }

    for (unsigned int i = 0; i < activeChannels; i++) {
        m_txTimeout[i] = new CTimer(1000, 1, 0);
    }

    LogMessage("Initializing SDRSoapyMulti");
}

CSDRSoapyMulti::~CSDRSoapyMulti() {
    for (unsigned int i = 0; i < m_activeChannels; i++) {
        delete m_rxIQBuffer[i];
        delete m_txIQBuffer[i];
        delete m_txTimeout[i];

        m_modems[i] = nullptr;
    }

    LogMessage("SDRSoapyMulti released");
}

void CSDRSoapyMulti::setIO(CIO* io, unsigned int ch) {
    if (ch > (sizeof(m_modems) / sizeof(m_modems[0])))
        return;

    m_modems[ch] = io;
}

CIO* CSDRSoapyMulti::getIO(unsigned int ch) {
    if (ch > (sizeof(m_modems) / sizeof(m_modems[0])))
        return nullptr;

    return m_modems[ch];
}


bool CSDRSoapyMulti::start(bool trace) {
    if (m_started) {
        LogDebug("SDRSoapyMulti alreay started, start abort");
        return true;
    }

    m_trace = trace;

    m_started = true;
    LogMessage("Soapy SDR Multi start");

    return true;
}

bool CSDRSoapyMulti::startSDRInt() {
    assert(m_rxFreq > 0);
    assert(m_txFreq > 0);
    assert(m_sampleRate > 0);

    // Initlize the IO buffers
    for (unsigned int i = 0; i < MAX_MMDVM_CHANNELS; i++) {
        delete m_rxIQBuffer[i];
        delete m_txIQBuffer[i];
        m_rxIQBuffer[i] = nullptr;
        m_txIQBuffer[i] = nullptr;
    }

    for (unsigned int i = 0; i < m_activeChannels; i++) {
        std::string name = "RXIQBuffer[" + std::to_string(i) + "]";
        m_rxIQBuffer[i] = new CRingBuffer<RXSample>(722, name.c_str());

        name = "TXIQBuffer[" + std::to_string(i) + "]";
        m_txIQBuffer[i] = new CRingBuffer<TXSample>(722, name.c_str());
    }

    // SDR Device
    float rx_gain = float(m_conf->getRxGain());
    float tx_gain = float(m_conf->getTxGain());;
    float rx_freq = (float)m_rxFreq - DEFAULT_BASEBAND_SHIFT;
    float tx_freq = (float)m_txFreq - DEFAULT_BASEBAND_SHIFT;
    // float pocsag_freq = (float)m_pocsagFreq - DEFAULT_BASEBAND_SHIFT;

    // TODO - determin fro config
    std::string rx_antenna = m_conf->getRxAntenna();    // "LNAH";
    std::string tx_antenna = m_conf->getTxAntenna();    // "BAND1";
    std::string deviceType = m_conf->getModemType();    // "limesdr"
    std::string modemURI   = m_conf->getModemURI();

    bool needs_timestamp = true;
    if (deviceType.compare("plutosdr") == 0 || deviceType.compare("pluto") == 0)
        needs_timestamp = false;

    LogMessage("SDR Device Parameters");
    LogMessage("  Device Type:      %s", deviceType.c_str());
    LogMessage("  Device URI:       %s", modemURI.c_str());
    LogMessage("  Sample Rate:      %u samples/sec", m_sampleRate);
    LogMessage("  RX Frequency:     %.0f Hz", rx_freq);
    LogMessage("  TX Frequency:     %.0f Hz", tx_freq);
    LogMessage("  RX Gain:          %.0f", rx_gain);
    LogMessage("  TX Gain:          %.0f", tx_gain);
    LogMessage("  RX Antenna:       %s", rx_antenna.c_str());
    LogMessage("  TX Antenna:       %s", tx_antenna.c_str());
    // LogMessage("  POCSAG Frequency: %.0f Hz", m_soapyPocsagFreq);
    LogMessage("  PFB Channels:     %u", m_pfbChannels);
    LogMessage("  Active Channels:  %u", m_activeChannels);

    CSoapyDevice* device = new CSoapyDevice(deviceType, modemURI, double(m_sampleRate), rx_freq, tx_freq,
                                rx_gain, tx_gain, rx_antenna, tx_antenna, m_pfbChannels, m_trace);
    if (!device->getSoapyInit() || (device->getRxStream() == nullptr) || (device->getTxStream() == nullptr)) {
        LogError("Initialize SoapySDR failed");
        delete device;
        return false;
    }
    m_device = device;

    // Transmitter/Receiver
    int sample_delay = 10;                  // conf.getSampleDelay(); // can be negative
    unsigned int rf_delay = 10;             // conf.getRFDelay();
    unsigned int digital_gain = 35;         // std::max<unsigned int>(conf.getDigitalGain(), 1U);
    float dac_scaling = std::min<float>(float(digital_gain) / 100.0f, MAX_TX_DAC_SCALE);
    float symbol_deviation = 10;            // float(std::max<unsigned int>(conf.getSymbolDeviation(), 1U));
    unsigned int power_calibration = 70;    // conf.getRSSICalibration();

    LogMessage("Tranceiver Parameters");
    LogMessage("  Sample Delay:     %u", sample_delay);
    LogMessage("  RF Delay:         %u", rf_delay);
    LogMessage("  Digital Gain:     %u", digital_gain);
    LogMessage("  Symbol Deviation: %.0f", symbol_deviation);
    LogMessage("  Power Cal:        %u", power_calibration);

    // DMR Timing Controller
    m_dmrTiming = new CDMRTiming(rf_delay, sample_delay);

    // Run the device rx/tx threads
    m_receiver = new CReceiver(this, m_device, m_dmrTiming, 
                                m_activeChannels, m_pfbChannels, float(m_sampleRate), 
                                needs_timestamp, symbol_deviation, power_calibration);

    m_transmitter = new CTransmitter(this, m_device, m_dmrTiming, 
                                      m_activeChannels, m_pfbChannels, float(m_sampleRate),
                                      needs_timestamp, symbol_deviation, dac_scaling);

    m_receiver->run();
    m_transmitter->run();

    return true;
}

void CSDRSoapyMulti::process(unsigned int ch) {

    if (!m_started)
        return;

    if (m_applyParams && ch == 0) {
        LogMessage("SDRSoapyMulti applying parameters...");

        // reinit the SDR devices
        this->stopSDRInt();

        bool success = this->startSDRInt();
        if (!success)
            LogError("Error startup SoapySDR device");

        m_applyParams = false;
    }

    if (!m_device)
        return;

    CModem& modem = getIO(ch)->getModem();

    // TX flag reset timer
    m_txTimeout[ch]->clock();
    if (m_txTimeout[ch]->hasExpired()) {
        if (modem.m_tx) {
            modem.m_tx = false;
            LogMessage("SDRSoapyMulti TX off (timeout)");
        }

        m_txTimeout[ch]->stop();
    }

    // Switch off the transmitter if needed
    auto txBuffer = m_txIQBuffer[ch];
    if (txBuffer->hasData() && !modem.m_tx) {
        LogDebug("SDRSoapyMulti discarded TX samples %u, ch %u", txBuffer->dataSize(), ch);
        txBuffer->clear();  // clear off partial DMR timeslot data so good timing info is present in packet
    }

    // The Transmitter will auto stop on no data
    // if (!txBuffer->hasData() && modem.m_tx) {
    //     modem.m_tx = false;
    //     LogMessage("SDRSoapyMulti TX OFF (out)");
    // }
}

void CSDRSoapyMulti::stop() {
    if (!m_started) {
        LogDebug("SDRSoapyMulti is not started yet, stop abort");
        return;
    }

    stopSDRInt();

    m_started = false;

    LogDebug("SDRSoapyMulti stopped");
}

void CSDRSoapyMulti::stopSDRInt() {    
    if (m_receiver)
        m_receiver->stop();

    if(m_transmitter)
        m_transmitter->stop();

    delete m_receiver;
    m_receiver = nullptr;

    delete m_transmitter;
    m_transmitter = nullptr;

    delete m_dmrTiming;
    m_dmrTiming = nullptr;

    delete m_device;
    m_device = nullptr;
}

//
// SDRDevice interface for MMDVM-IO core code
//
// read max 2 rx samples from device to IQ to demod
int CSDRSoapyMulti::read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control, unsigned int ch) {

    if (!m_started)
        return 0;

    if (ch >= m_activeChannels)
        return 0;

    CModem& modem = getIO(ch)->getModem();

    int ret = 0;
    // Reads from rx iq buffer
    if (m_rxIQBuffer[ch] && m_rxIQBuffer[ch]->dataSize() >= RX_BLOCK_SIZE) {

        m_rxMutex.lock();
        if (m_rxIQBuffer[ch]->dataSize() >= RX_BLOCK_SIZE) {
            RXSample rxSample;
            for (unsigned int i = 0; i < RX_BLOCK_SIZE; i++) {
                m_rxIQBuffer[ch]->getData(rxSample);

                samples[i]  = rxSample.m_sample;
                control[i]  = rxSample.m_control;
                rssi[i]     = rxSample.m_rssi;
                ret = RX_BLOCK_SIZE;

                // Mute the receiver when transmitting in simplex mode
                if (modem.m_tx && !modem.m_duplex) {
                    samples[i] = 0;
                    control[i] = 0;
                    rssi[i] = 0;
                }
            }
        }
        m_rxMutex.unlock();

        return ret;
    }

    return ret;
}

void CSDRSoapyMulti::write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control, unsigned int ch)
{
    assert(samples != nullptr);
    assert(length > 0U);

    if (!m_started)
        return;

    if (m_applyParams)
        return;

    CModem& modem = getIO(ch)->getModem();

    if (!modem.m_tx) {
        modem.m_tx = true;
        LogMessage("SoapySDRMulti TX ON");

        m_txTimeout[ch]->start();
    }

    if (!m_txIQBuffer[ch])
        return;

    //   if (m_tx) {
    //     // Set the correct transmit frequency for the mode if needed, even in the middle of a transmission
    //     setTXFrequency(mode == MMDVM_STATE::POCSAG);
    //   }

    const q15_t LEVEL_50PC_INVERTED = -128 * 128;
    const q15_t LEVEL_40PC_INVERTED = -102 * 128;
    const q15_t LEVEL_30PC_INVERTED = -77  * 128;
    const q15_t LEVEL_100PC         =  255 * 128;

    q15_t txLevel;
    switch (mode) {
    case MMDVM_STATE::FM:
        txLevel = LEVEL_100PC;
        break;
    default:
        txLevel = LEVEL_40PC_INVERTED;
        break;
    }

    // Update the tx iq buffer

    m_txMutex.lock();

    for (uint16_t i = 0U; i < length; i++) {
        q31_t res1 = samples[i] * txLevel;
        q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));

        if (control == nullptr)
            m_txIQBuffer[ch]->addData({res2, MARK_NONE});
        else
            m_txIQBuffer[ch]->addData({res2, control[i]});
    }

    m_txMutex.unlock();

//   LogDebug("SoapySDR Multi write tx sample %u", length);
}

uint16_t CSDRSoapyMulti::getTXSpace(unsigned int ch) const {
    if (ch >= m_activeChannels)
        return 0;

    // LogDebug("SDRSOapyMulti TX Space %u", m_txIQBuffer[ch]->freeSpace());
    return m_txIQBuffer[ch] ? m_txIQBuffer[ch]->freeSpace() : 0;
}

uint8_t CSDRSoapyMulti::setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
  if ((txFreq < MIN_RF_FREQUENCY) || (txFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((rxFreq < MIN_RF_FREQUENCY) || (rxFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((pocsagFreq < MIN_RF_FREQUENCY) || (pocsagFreq > MAX_RF_FREQUENCY))
    return 4U;

  m_power      = float(power) / 255.0F;
  m_txFreq     = txFreq;
  m_rxFreq     = rxFreq;
  m_pocsagFreq = pocsagFreq;

  LogDebug("SDR Soapy Multi setFrequency() %u/%u", txFreq, rxFreq);
  return 0U;
}

void CSDRSoapyMulti::setChannels(unsigned int pfbChannels, unsigned int activeChannels) {
    assert(m_sampleRate > 0);


    // Max PFB channels [6 ~ 10], determined by SDR sample rate
    unsigned int maxPFBChannels = m_sampleRate / 25000;

    if (pfbChannels < 3)
        pfbChannels = 3;
    
    if (pfbChannels > maxPFBChannels)
        pfbChannels = maxPFBChannels;

    // max active channels between [1 ~ pfbChannels], not exceeding 7
    unsigned int maxActiveChannels = std::min(pfbChannels, MAX_MMDVM_CHANNELS);
    if (activeChannels > maxActiveChannels)
        activeChannels = maxActiveChannels;
    else if (activeChannels == 0)
        activeChannels = 1;

    m_pfbChannels = pfbChannels;
    m_activeChannels = activeChannels;

    LogMessage("SDRSoapyMulti setChannels: pfb: %u, act: %u", m_pfbChannels, m_activeChannels);
}

uint8_t CSDRSoapyMulti::setParameters() {
    // Will apply parameters in the runloop
    m_applyParams = true;
    return 0U;
}

//
// ReceiverDelegate
//
void CSDRSoapyMulti::onRXSamples(const std::vector<uint8_t> &controlBuf, const std::vector<int16_t>& sampleBuf, unsigned int rssi, unsigned int channel) {

    // calculate the rx samplerate (720 frames per slot)
    LOG_SAMPLE_INTERVAL("SDRSoapyMulti RX Samples", SAMPLES_PER_SLOT, channel);

    // trigger tx consuming buffered data
    // m_transmitter->notifyTXDataUpdate(channel);

    if (channel >= m_activeChannels)
        return;

    if (!m_rxIQBuffer[channel])
        return;

    // always write 720 frames to MMDVM-IQ RXSamples
    RXSample rxsample;

    m_rxMutex.lock();

    if (m_rxIQBuffer[channel]->freeSpace() < SAMPLES_PER_SLOT) {
        LogDebug("DMRSoapy Multi dropped %u bytes previous RX samples, ch %u", m_rxIQBuffer[channel]->dataSize(), channel);
        m_rxIQBuffer[channel]->clear();
    }

    for (unsigned int i = 0; i < SAMPLES_PER_SLOT; i++) {
        rxsample.m_control = controlBuf[i];
        rxsample.m_sample = sampleBuf[i];
        rxsample.m_rssi = rssi;
        // rxsample.m_ch = channel;

        m_rxIQBuffer[channel]->addData(rxsample);
    }

    m_rxMutex.unlock();
}

//
// TransmitterDelegate
//
int CSDRSoapyMulti::getTXSamples(std::vector<uint8_t> &controlBuf, std::vector<float> &sampleBuf, float symbolDeviation, unsigned int channel) {
    LOG_SAMPLE_INTERVAL("SDRSoapyMulti Polling TX Samples", SAMPLES_PER_SLOT, channel);

    if (channel >= m_activeChannels)
        return 0;

    int ret = 0;

    // requires at least 720 samples to process
    if (m_txIQBuffer[channel]->dataSize() >= SAMPLES_PER_SLOT) {
        m_txMutex.lock();

        if (m_txIQBuffer[channel]->dataSize() >= SAMPLES_PER_SLOT) {
            m_txTimeout[channel]->start();

            // Convert to SDR-Multi data formats (720 control and samples)
            for (unsigned int i = 0; i < SAMPLES_PER_SLOT; i++) {
                TXSample txSample = {0, 0};
                m_txIQBuffer[channel]->getData(txSample);
                uint8_t control = txSample.m_control;
                float samplef  = (float(txSample.m_sample) * symbolDeviation) / 32767.0f;
                controlBuf.push_back(control);
                sampleBuf.push_back(samplef);
            }

            ret =  SAMPLES_PER_SLOT;
        }

        m_txMutex.unlock();

        if (ret > 0) {
            LogDebug("SDRSoapyMulti TX samples %u, left %u, ch %u", ret, m_txIQBuffer[channel]->dataSize(), channel);
        }

    }  else {
        // // filling empty frames
        // TXSample txSample = {0, 0};
        // for (unsigned int i = 0; i < SAMPLES_PER_SLOT; i++) {
        //     uint8_t control = txSample.m_control;
        //     int16_t sample  = float(txSample.m_sample) * symbolDeviation / 32767.0f;
        //     controlBuf.push_back(control);
        //     sampleBuf.push_back(sample);
        // }

        // LogDebug("SDRSoapyMulti filling %u samples, buffered %u, ch[%u]", SAMPLES_PER_SLOT, m_txIQBuffer[ch]->dataSize(), ch);
    }

    return ret;
}

#endif