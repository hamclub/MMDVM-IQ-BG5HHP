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
#if !defined(SDRDEVICE_MULTI_H)
#define  SDRDEVICE_MULTI_H

// #if defined(USE_SOAPY_MULTI)
// #endif

#include "SDRDevice.h"

#include "RingBuffer.h"
#include "Timer.h"

#include "Constants.h"
#include "SoapyDevice.h"
#include "DMRTiming.h"
#include "Receiver.h"
#include "Transmitter.h"

#include <vector>

class CConf;

class CSDRSoapyMulti : public ISDRDevice, public ReceiverDelegate, public TransmitterDelegate {
public:
    CSDRSoapyMulti(CConf* conf);
    virtual ~CSDRSoapyMulti();

    virtual bool start(bool trace);
    virtual void process(unsigned int ch);
    virtual void stop();

    virtual void write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control, unsigned int ch);
    virtual int read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control, unsigned int ch);

    virtual uint16_t getTXSpace(unsigned int channel) const;

    virtual uint8_t setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);
    virtual void setChannels(unsigned int pfbChannels, unsigned int activeChannels);

    virtual uint8_t setParameters();

public:
    // Interface ReceiverDelegate, TransmitterDelegate
    virtual void onRXSamples(const std::vector<uint8_t> &controlBuf, const std::vector<int16_t>& sampleBuf, unsigned int rssi, unsigned int channel);
    virtual int getTXSamples(std::vector<uint8_t> &controlBuf, std::vector<float> &sampleBuf, float symbolDeviation, unsigned int channel);

public:
    virtual void setIO(CIO* io, unsigned int ch);
    CIO* getIO(unsigned int ch);

private:
    bool startSDRInt();
    void stopSDRInt();

private:
    CConf*          m_conf;

    unsigned int    m_sampleRate        = 0;
    unsigned int    m_pfbChannels       = 3;
    unsigned int    m_activeChannels    = 1;

    float           m_power             = 0.f;
    uint32_t        m_txFreq            = 0;
    uint32_t        m_rxFreq            = 0;
    uint32_t        m_pocsagFreq        = 0;
    
    bool            m_trace             = false;
    volatile bool   m_started           = false;

    CSoapyDevice*    m_device            = nullptr;
    CDMRTiming*      m_dmrTiming         = nullptr;

    CReceiver*       m_receiver          = nullptr;
    CTransmitter*    m_transmitter       = nullptr;

    volatile bool   m_applyParams       = false;

    CTimer         *m_txTimeout[MAX_MMDVM_CHANNELS];

    // Buffer mutex
    CMutex      m_rxMutex;
    CMutex      m_txMutex;

    // Buffer for MMDVM-IQ
    CRingBuffer<RXSample> *m_rxIQBuffer[MAX_MMDVM_CHANNELS];
    CRingBuffer<TXSample> *m_txIQBuffer[MAX_MMDVM_CHANNELS];

    // IO holder
    CIO *m_modems[MAX_MMDVM_CHANNELS];
};

#endif