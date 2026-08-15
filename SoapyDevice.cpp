/*
 *   Copyright (C) 2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2026 by Adrian Musceac YO8RZZ
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

#include "SoapyDevice.h"
#include "Log.h"

#include <cassert>

const size_t RX_CHANNEL = 0;
const size_t TX_CHANNEL = 0;

CSoapyDevice::CSoapyDevice(std::string deviceType, std::string modemURI, double sampleRate, float rxFreq, float txFreq,
               float rxGain, float txGain, std::string rxAntenna, std::string txAntenna,
               unsigned int num_pfb_channels, bool debug) :
m_soapyDeviceType(deviceType),
m_soapyDeviceURI(modemURI),
m_type(SOAPY_TYPE::NONE),
m_device(nullptr),
m_rxStream(nullptr),
m_txStream(nullptr),
m_sampleRate(sampleRate),
m_soapyTXFreq(txFreq),
m_soapyRXFreq(rxFreq),
m_soapyTXGain(txGain),
m_soapyRXGain(rxGain),
m_rxAntenna(rxAntenna),
m_txAntenna(txAntenna),
m_soapyInit(false)
{
    SoapySDR::Kwargs devArgs;
    SoapySDR::Kwargs rxArgs;
    SoapySDR::Kwargs txArgs;

    if (debug)
        SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
    else
        SoapySDR::setLogLevel(SOAPY_SDR_INFO);

    const char* LIME_DEFAULT_URI  = "index=0";         // eg: addr=1111:2222 or serial=xxxxxxxx
    const char* PLUTO_DEFAULT_URI = "ip:pluto.local";

    if (m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0) {
        const char* uri = m_soapyDeviceURI.empty() ? PLUTO_DEFAULT_URI : m_soapyDeviceURI.c_str();
        std::string rxBufLen = std::to_string(RX_INTERP_IN_SIZE * num_pfb_channels);
        std::string txBufLen = std::to_string(TX_INTERP_OUT_SIZE * num_pfb_channels);
        devArgs["driver"] = "plutosdr";
        rxArgs["uri"]     = uri;
        txArgs["uri"]     = uri;
        rxArgs["bufflen"] = rxBufLen.c_str();
        txArgs["bufflen"] = txBufLen.c_str();
        ::LogInfo("Using Pluto SDR driver uri %s", uri);
        m_type = SOAPY_TYPE::PlutoSDR;
    } else if (m_soapyDeviceType.compare("limesdr") == 0  || m_soapyDeviceType.compare("lime") == 0 ||
               m_soapyDeviceType.compare("limemini") == 0 || m_soapyDeviceType.compare("limenet-micro") == 0) {
        const char* uri = m_soapyDeviceURI.empty() ? LIME_DEFAULT_URI : m_soapyDeviceURI.c_str();
        devArgs["driver"] = "lime";
        rxArgs["uri"]     = uri;
        txArgs["uri"]     = uri;
        rxArgs["latency"] = "0";
        txArgs["latency"] = "0";
        ::LogInfo("Using Lime SDR driver uri %s", uri);
        m_type = SOAPY_TYPE::LimeSDR;
    } else if (m_soapyDeviceType.compare("usrp") == 0) {
        const char* uri = m_soapyDeviceURI.c_str();
        devArgs["driver"] = "uhd";
        rxArgs["uri"]     = uri;
        txArgs["uri"]     = uri;
        rxArgs["recv_frame_size"] = "1024";
        ::LogInfo("Using Ettus USRP driver uri %s", uri);
        m_type = SOAPY_TYPE::USRP;
    } else if (m_soapyDeviceType.compare("mucell") == 0) {
        devArgs["driver"] = "mucell";
        ::LogInfo("Using the muCell driver");
        m_type = SOAPY_TYPE::MuCell;
    } else {
        devArgs["driver"] = "sx";
        ::LogInfo("Using the SXCeiver driver");
        m_type = SOAPY_TYPE::SXceiver;
    }

    try {
        m_device = SoapySDR::Device::make(devArgs);
        assert(m_device != nullptr);
    
        m_device->setSampleRate(SOAPY_SDR_RX, RX_CHANNEL, m_sampleRate);
        m_device->setSampleRate(SOAPY_SDR_TX, TX_CHANNEL, m_sampleRate);
    
        m_device->setFrequency(SOAPY_SDR_RX, RX_CHANNEL, m_soapyRXFreq);
        m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXFreq);
    
        m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, m_rxAntenna);
        m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, m_txAntenna);

        // Device TX calibration routine requires normal gains for RF loopback
        m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_soapyRXGain);
        m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXGain);

        m_device->setBandwidth(SOAPY_SDR_RX, RX_CHANNEL, m_sampleRate);
        m_device->setBandwidth(SOAPY_SDR_TX, TX_CHANNEL, m_sampleRate);

        m_rxStream = m_device->setupStream(SOAPY_SDR_RX, "CF32", {RX_CHANNEL}, rxArgs);
        m_txStream = m_device->setupStream(SOAPY_SDR_TX, "CF32", {TX_CHANNEL}, txArgs);

        m_device->activateStream(m_txStream);
        m_device->activateStream(m_rxStream);
    
        assert(m_rxStream != nullptr);
        assert(m_txStream != nullptr);

        m_rxMTU = m_device->getStreamMTU(m_rxStream);
        m_txMTU = m_device->getStreamMTU(m_txStream);

        SoapySDR::Range txGainRange = m_device->getGainRange(SOAPY_SDR_TX, TX_CHANNEL);
        SoapySDR::Range rxGainRange = m_device->getGainRange(SOAPY_SDR_RX, RX_CHANNEL);
        m_minTxGain = float(txGainRange.minimum()) + float(txGainRange.step());

        ::LogMessage("Soapy device initialized");
        ::LogMessage("  RX stream MTU is %u", m_rxMTU);
        ::LogMessage("  TX stream MTU is %u", m_txMTU);
        ::LogMessage("  Minimum TX gain: %f dB", txGainRange.minimum());
        ::LogMessage("  Maximum TX gain: %f dB", txGainRange.maximum());
        ::LogMessage("  Minimum RX gain: %f dB", rxGainRange.minimum());
        ::LogMessage("  Maximum RX gain: %f dB", rxGainRange.maximum());
        m_soapyInit = true;
    } catch (std::runtime_error& e) {
        ::LogError("Soapy device failed to initialize");
    }
}

CSoapyDevice::~CSoapyDevice()
{
    if (m_device != nullptr) {
        assert(m_rxStream != nullptr);
        assert(m_txStream != nullptr);
    
        if (m_soapyInit) {
            m_device->deactivateStream(m_rxStream, 0, 0);
            m_device->deactivateStream(m_txStream, 0, 0);
        }
    
        m_device->closeStream(m_rxStream);
        m_device->closeStream(m_txStream);
    
        SoapySDR::Device::unmake(m_device);
    }

    ::LogMessage("Soapy device closed");

    m_rxStream = nullptr;
    m_txStream = nullptr;
    m_device   = nullptr;
}

bool CSoapyDevice::getSoapyInit() const
{
    return m_soapyInit;
}

SoapySDR::Device* CSoapyDevice::getDevice()
{
    return m_device;
}

unsigned int CSoapyDevice::getRxMTU() const
{
    return m_rxMTU;
}

unsigned int CSoapyDevice::getTxMTU() const
{
    return m_txMTU;
}

void CSoapyDevice::setTx(bool tx)
{
    if (tx) {
        ::LogMessage("Device: TX ON");

        if ((m_type == SOAPY_TYPE::MuCell) || (m_type == SOAPY_TYPE::SXceiver))
            m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "TX");
        else
            m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXGain);
    } else {
        ::LogMessage("Device: TX OFF");

        if ((m_type == SOAPY_TYPE::MuCell) || (m_type == SOAPY_TYPE::SXceiver))
            m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "NONE");
        else
            m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_minTxGain);
    }

    // poke GPIO/relay code here
}

#endif