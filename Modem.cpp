/*
 *   Copyright (C) 2025,2026 by Jonathan Naylor G4KLX
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

#include "Modem.h"
#include "Globals.h"
#include "Utils.h"
#include "FDDC.h"
#include "FDUC.h"

#include <cassert>
#include <cstdio>
#include <cmath>

#if !defined(M_PI)
const float M_PI = 3.141592654F;
#endif

const uint8_t FRAME_START = 0xC0U;

const uint8_t TYPE_GET_VERSION              = 0x00U;
const uint8_t TYPE_SET_FREQUENCY_AND_POWER  = 0x01U;
const uint8_t TYPE_START                    = 0x02U;
const uint8_t TYPE_STOP                     = 0x03U;
const uint8_t TYPE_GET_STATUS               = 0x04U;
const uint8_t TYPE_TRANSMIT_DATA            = 0x05U;
const uint8_t TYPE_RECEIVE_DATA             = 0x06U;
const uint8_t TYPE_ACK                      = 0xFEU;
const uint8_t TYPE_NAK                      = 0xFFU;

const uint8_t CAP_DUPLEX                    = 0x01U;
const uint8_t CAP_HAS_TRANSMIT              = 0x02U;
const uint8_t CAP_HAS_RECEIVE               = 0x04U;

const uint8_t STATUS_TX_ON                  = 0x01U;

const uint8_t ERR_UNKNOWN_MESSAGE_TYPE       = 0x00U;
const uint8_t ERR_MALFORMED_MESSAGE          = 0x01U;
const uint8_t ERR_TRANSMIT_DATA_BEFORE_START = 0x02U;
const uint8_t ERR_TRANSMIT_BUFFER_OVERFLOW   = 0x03U;
const uint8_t ERR_INVALID_FREQUENCY          = 0x04U;
const uint8_t ERR_TRANSMIT_DATA_ERROR        = 0x05U;
const uint8_t ERR_OUT_OF_MEMORY              = 0x07U;

const int32_t DEVIATION = 550000;               // TODO

const uint8_t  GET_VERSION_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_GET_VERSION};
const uint16_t GET_VERSION_MESSAGE_LEN = sizeof(GET_VERSION_MESSAGE) / sizeof(uint8_t);

const uint8_t  GET_STATUS_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_GET_STATUS};
const uint16_t GET_STATUS_MESSAGE_LEN = sizeof(GET_STATUS_MESSAGE) / sizeof(uint8_t);

const uint8_t  STOP_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_STOP};
const uint16_t STOP_MESSAGE_LEN = sizeof(STOP_MESSAGE) / sizeof(uint8_t);

const uint16_t BB_ELEMENT_LEN   = sizeof(uint16_t);
const uint16_t IQ8_ELEMENT_LEN  = sizeof(int8_t) + sizeof(int8_t);
const uint16_t IQ12_ELEMENT_LEN = sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);
const uint16_t IQ16_ELEMENT_LEN = sizeof(int16_t) + sizeof(int16_t);

CModem* CModem::m_ptr = nullptr;


CModem::CModem() :
m_modem(),
m_state(MODEM_STATE::NONE),
m_txBuffer(nullptr),
m_rxBuffer(nullptr),
m_rxPtr(0U),
m_rxLen(0U),
m_statusTimer(1000U, 0U, 200U),
m_messageTimer(1000U, 0U, 100U),
m_watchdogTimer(1000U, 0U, 800U),
m_transmitTimer(1000U, 0U, 200U),
m_power(0U),
m_txFreq(0U),
m_rxFreq(0U),
m_pocsagFreq(0U),
m_duplex(false),
m_hasTX(false),
m_hasRX(false),
m_sampleRate(0U),
m_format(MODEM_FORMAT::NONE),
m_maxTXSamples(0U),
m_fdc24RX(nullptr),
m_fdc24TX(nullptr),
m_fdc72RX(nullptr),
m_fdc72TX(nullptr),
m_toModem(5000U, "Modem output buffer"),
m_spaceLeft(0U),
m_tx(false),
m_phase(0U),
m_lastPhase(0),
m_lastI24(0.0F),
m_lastQ24(0.0F),
m_lastI72(0.0F),
m_lastQ72(0.0F),
m_stopwatch(),
m_trace(false)
{
    // The largest needed
    m_rxBuffer = new uint8_t[MAX_RX_SAMPLES * IQ16_ELEMENT_LEN + 20U];

    m_ptr = this;
}

CModem::~CModem()
{
    delete[] m_txBuffer;
    delete[] m_rxBuffer;

    delete m_fdc24RX;
    delete m_fdc24TX;
    delete m_fdc72RX;
    delete m_fdc72TX;
}

void CModem::setUARTConnection(const std::string& port, unsigned int speed, bool trace)
{
    assert(!port.empty());
    assert(speed > 0U);

    m_trace = trace;

    m_modem.setUARTConnection(port, speed, trace);
}

void CModem::setUDPConnection(const std::string& modemAddress, uint16_t modemPort, const std::string& localAddress, uint16_t localPort, bool trace)
{
    assert(!modemAddress.empty());
    assert(!localAddress.empty());
    assert(modemPort > 0U);
    assert(localPort > 0U);

    m_trace = trace;

    m_modem.setUDPConnection(modemAddress, modemPort, localAddress, localPort, trace);
}

void CModem::setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
    m_power      = power;
    m_txFreq     = txFreq;
    m_rxFreq     = rxFreq;
    m_pocsagFreq = pocsagFreq;

    writeSetFreqPower();
}

bool CModem::hasDuplex() const
{
    return m_duplex;
}

bool CModem::hasTX() const
{
    return m_hasTX;
}

bool CModem::hasRX() const
{
    return m_hasRX;
}

bool CModem::canTETRA() const
{
    return (m_format == MODEM_FORMAT::IQ_8)  ||
           (m_format == MODEM_FORMAT::IQ_12) ||
           (m_format == MODEM_FORMAT::IQ_16);
}

bool CModem::start()
{
    bool ret = m_modem.open();
    if (!ret)
        return false;

    m_spaceLeft = 0U;

    m_state = MODEM_STATE::WAIT_VERSION;

    m_stopwatch.start();
    m_messageTimer.start();

    return true;
}

// Called from the main loop
void CModem::process()
{
    // Handle the receive data
    uint8_t c;
	while (m_modem.read(&c, 1U) > 0U) {
        if (m_rxPtr == 0U) {
            if (c == FRAME_START) {
                // Handle the frame start correctly
                m_rxBuffer[0U] = c;
                m_rxPtr = 1U;
                m_rxLen = 0U;
            } else {
                m_rxPtr = 0U;
                m_rxLen = 0U;
            }
        } else if (m_rxPtr == 1U) {
            // Handle the frame length, 1/2
            m_rxLen = c;
            m_rxBuffer[m_rxPtr] = c;
            m_rxPtr = 2U;
        } else if (m_rxPtr == 2U) {
            // Handle the frame length, 2/2
            m_rxLen += uint16_t(c) * 256U;
            m_rxBuffer[m_rxPtr] = c;
            m_rxPtr = 3U;
        } else {
            // Any other bytes are added to the buffer
            m_rxBuffer[m_rxPtr] = c;
            m_rxPtr++;

            // The full packet has been received, process it
            if (m_rxPtr == m_rxLen) {
                if (m_trace)
                    dump("Read Modem", m_rxBuffer, m_rxLen);

                processMessage(m_rxBuffer[3U], m_rxBuffer + 4U, m_rxLen - 4U);
                m_rxPtr = 0U;
                m_rxLen = 0U;
            }
        }
    }

    // Is this correct?
    unsigned int ms = m_stopwatch.elapsed();
    m_statusTimer.clock(ms);
    m_messageTimer.clock(ms);
    m_watchdogTimer.clock(ms);
    m_transmitTimer.clock(ms);
    m_stopwatch.start();

    if (m_messageTimer.isRunning() && m_messageTimer.hasExpired()) {
        switch (m_state) {
        case MODEM_STATE::NONE:
            m_messageTimer.stop();
            break;
        case MODEM_STATE::WAIT_VERSION:
            writeGetVersion();
            m_messageTimer.start();
            break;
        case MODEM_STATE::WAIT_FREQ_POWER:
            writeSetFreqPower();
            m_messageTimer.start();
            break;
        case MODEM_STATE::WAIT_START:
            writeStart();
            m_messageTimer.start();
            break;
        case MODEM_STATE::WAIT_STOP:
            writeStop();
            m_messageTimer.start();
            break;
        default:	// SMS_RUNNING
            m_messageTimer.stop();
            break;
        }
    }

    // Handle the transmit data & status
    if (m_state == MODEM_STATE::RUNNING) {
        switch (m_format) {
        case MODEM_FORMAT::BASEBAND:
            writeTransmitDataBB(false);
            break;
        case MODEM_FORMAT::IQ_8:
            writeTransmitDataIQ8(false);
            break;
        case MODEM_FORMAT::IQ_12:
            writeTransmitDataIQ12(false);
            break;
        case MODEM_FORMAT::IQ_16:
            writeTransmitDataIQ16(false);
            break;
        default:
            break;
        }
/*
        if (m_statusTimer.isRunning() && m_statusTimer.hasExpired()) {
            writeGetStatus();
            m_statusTimer.start();
        }
*/
    }

    if (m_transmitTimer.isRunning() && m_transmitTimer.hasExpired()) {
        LogWarning("The transmit timer has expired");
        // Handle the remaining transmit data
        switch (m_format) {
        case MODEM_FORMAT::BASEBAND:
            writeTransmitDataBB(true);
            break;
        case MODEM_FORMAT::IQ_8:
            writeTransmitDataIQ8(true);
            break;
        case MODEM_FORMAT::IQ_12:
            writeTransmitDataIQ12(true);
            break;
        case MODEM_FORMAT::IQ_16:
            writeTransmitDataIQ16(true);
            break;
        default:
            break;
        }
    }
/*
    if (m_watchdogTimer.isRunning() && m_watchdogTimer.hasExpired()) {
        LogWarning("The watchdog timer has expired");
        m_state = SERIALMODEM_STATE::NONE;
        // Actually not sure what to do here
    }
*/
}

// For all modes bar TETRA and P25 phase 2
bool CModem::writeSampleFSK24(uint8_t marker, q15_t frequency, uint8_t amplitude)
{
    if (m_state != MODEM_STATE::RUNNING)
        return false;

    if (m_sampleRate < 24U)
        return false;

    switch (m_format) {
    case MODEM_FORMAT::BASEBAND:
        return processSampleFSK24BB(marker, frequency, amplitude);

    case MODEM_FORMAT::IQ_8:
    case MODEM_FORMAT::IQ_12:
    case MODEM_FORMAT::IQ_16:
        return processSampleFSK24IQ(marker, frequency, amplitude);

    default:
        LogError("Unknown TX format for FSK24");
        return false;
    }
}

uint16_t CModem::getTXSpace() const
{
    return m_spaceLeft;
}

bool CModem::isTX() const
{
    return m_tx;
}

void CModem::processMessage(uint8_t type, const uint8_t* data, uint16_t length)
{
    assert(data != nullptr);

    switch (type) {
    case TYPE_GET_VERSION:
        if (m_state == MODEM_STATE::WAIT_VERSION) {
            LogDebug("Received Modem::GET_VERSION");

            bool ret = processVersion(data, length);
            if (!ret)
                throw;

            m_state = MODEM_STATE::NONE;
            m_messageTimer.start();
        }
        break;
    case TYPE_ACK:
        if (m_state == MODEM_STATE::WAIT_FREQ_POWER) {
            LogDebug("Received Modem::ACK");
            writeStart();
            m_messageTimer.start();
        } else if (m_state == MODEM_STATE::WAIT_START) {
            LogDebug("Received Modem::ACK");
            LogMessage("Modem has been started");
            m_state = MODEM_STATE::RUNNING;
            m_messageTimer.stop();
        }
        break;
    case TYPE_NAK:
        LogDebug("Received Modem::NAK 0x%02X", data[1U]);
        switch (data[1U]) {
        case ERR_UNKNOWN_MESSAGE_TYPE:
            LogMessage("Unknown message type for message type 0x%02X", data[0U]);
            break;
        case ERR_MALFORMED_MESSAGE:
            LogMessage("Malformed message received for message type 0x%02X", data[0U]);
            break;
        case ERR_TRANSMIT_DATA_BEFORE_START:
            LogMessage("Modem not started for message type 0x%02X", data[0U]);
            break;
        case ERR_TRANSMIT_BUFFER_OVERFLOW:
            LogMessage("Transmit buffer overflow for message type 0x%02X", data[0U]);
            break;
        case ERR_INVALID_FREQUENCY:
            LogMessage("Invalid frequency for the hardware for message type 0x%02X", data[0U]);
            break;
        case ERR_TRANSMIT_DATA_ERROR:
            LogMessage("Transmit data error for message type 0x%02X", data[0U]);
            break;
        case ERR_OUT_OF_MEMORY:
            LogMessage("Out of memory in the modem for message type 0x%02X", data[0U]);
            break;
        default:
            LogMessage("Unknown error type - 0x%02X for message type 0x%02X", data[1U], data[0U]);
            break;
        }
        break;
    case TYPE_GET_STATUS:
        m_spaceLeft = data[0U] + (data[1U] * 256U);
        m_tx = (data[2U] & STATUS_TX_ON) == STATUS_TX_ON;
        LogDebug("Received Modem::GET_STATUS space: %u TX: %u", m_spaceLeft, m_tx ? 1U : 0U);
        m_watchdogTimer.start();
        break;
    case TYPE_RECEIVE_DATA:
        if (m_state == MODEM_STATE::RUNNING) {
            uint8_t  marker = data[0U];
            uint16_t offset = data[1U] + (data[2U] * 256U);

            switch (m_format) {
            case MODEM_FORMAT::BASEBAND:
                LogDebug("Received Modem::RECEIVE_DATA BB Payload: %u %u", data[3U], (length - 4U) / BB_ELEMENT_LEN);
                processBBRX(marker, offset, data + 4U, length - 4U);
                break;
            case MODEM_FORMAT::IQ_8:
                LogDebug("Received Modem::RECEIVE_DATA 8-bit IQ Payload: %u %u", data[3U], (length - 4U) / IQ8_ELEMENT_LEN);
                processIQ8RX(marker, offset, data + 4U, length - 4U);
                break;
            case MODEM_FORMAT::IQ_12:
                LogDebug("Received Modem::RECEIVE_DATA 12-bit IQ Payload: %u %u", data[3U], (length - 4U) / IQ12_ELEMENT_LEN);
                processIQ12RX(marker, offset, data + 4U, length - 4U);
                break;
            case MODEM_FORMAT::IQ_16:
                LogDebug("Received Modem::RECEIVE_DATA 16-bit IQ Payload: %u %u", data[3U], (length - 4U) / IQ16_ELEMENT_LEN);
                processIQ16RX(marker, offset, data + 4U, length - 4U);
                break;
            default:
                break;
            }
        }
        break;
    default:
        LogMessage("Unknown message type - 0x%02X", type);
        break;
    }
}

void CModem::writeGetVersion()
{
    m_modem.write(GET_VERSION_MESSAGE, GET_VERSION_MESSAGE_LEN);

    if (m_trace)
        dump("Write Get Version", GET_VERSION_MESSAGE, GET_VERSION_MESSAGE_LEN);
    else
        LogDebug("Sent Modem::GET_VERSION");

    m_state = MODEM_STATE::WAIT_VERSION;
}

void CModem::writeSetFreqPower()
{
    uint8_t buffer[13U];

    buffer[0U] = FRAME_START;
    buffer[1U] = 0x0DU;
    buffer[2U] = 0x00U;
    buffer[3U] = TYPE_SET_FREQUENCY_AND_POWER;

    buffer[4U] = m_power;

    buffer[5U] = (m_rxFreq >> 0) & 0xFFU;       // LSB
    buffer[6U] = (m_rxFreq >> 8) & 0xFFU;
    buffer[7U] = (m_rxFreq >> 16) & 0xFFU;
    buffer[8U] = (m_rxFreq >> 24) & 0xFFU;      // MSB

    buffer[9U]  = (m_txFreq >> 0) & 0xFFU;      // LSB
    buffer[10U] = (m_txFreq >> 8) & 0xFFU;
    buffer[11U] = (m_txFreq >> 16) & 0xFFU;
    buffer[12U] = (m_txFreq >> 24) & 0xFFU;     // MSB

    m_modem.write(buffer, 13U);

    if (m_trace)
        dump("Write Set Freq/Power", buffer, 13U);
    else
        LogDebug("Sent Modem::SET_FREQUENCY_AND_POWER");

    m_state = MODEM_STATE::WAIT_FREQ_POWER;
}

void CModem::writeGetStatus()
{
    m_modem.write(GET_STATUS_MESSAGE, GET_STATUS_MESSAGE_LEN);

    if (m_trace)
        dump("Write Get Status", GET_STATUS_MESSAGE, GET_STATUS_MESSAGE_LEN);
    else
        LogDebug("Sent Modem::GET_STATUS");
}

void CModem::writeStart()
{
    uint8_t buffer[6U];

    buffer[0U] = FRAME_START;
    buffer[1U] = 0x06U;
    buffer[2U] = 0x00U;
    buffer[3U] = TYPE_START;

    buffer[4U] = MAX_RX_SAMPLES % 256U;     // LSB
    buffer[5U] = MAX_RX_SAMPLES / 256U;     // MSB

    m_modem.write(buffer, 6U);

    if (m_trace)
        dump("Write Start", buffer, 6U);
    else
        LogDebug("Sent Modem::START");

    m_state = MODEM_STATE::WAIT_START;

    m_watchdogTimer.start();
    m_statusTimer.start();
}

void CModem::writeStop()
{
    m_modem.write(STOP_MESSAGE, STOP_MESSAGE_LEN);

    if (m_trace)
        dump("Write Stop", STOP_MESSAGE, STOP_MESSAGE_LEN);
    else
        LogDebug("Sent Modem::STOP");

    m_state = MODEM_STATE::WAIT_STOP;

    m_toModem.clear();

    m_watchdogTimer.stop();
    m_transmitTimer.stop();
    m_statusTimer.stop();
}

bool CModem::processVersion(const uint8_t* data, uint16_t length)
{
    m_duplex = (data[1U] & CAP_DUPLEX)       == CAP_DUPLEX;
    m_hasTX  = (data[1U] & CAP_HAS_TRANSMIT) == CAP_HAS_TRANSMIT;
    m_hasRX  = (data[1U] & CAP_HAS_RECEIVE)  == CAP_HAS_RECEIVE;

    m_sampleRate = data[2U];

    if (m_sampleRate > 24U) {
        LogMessage("Instantiating the 24 kHz downsampler");
        m_fdc24RX = new CFDDC(24U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackRX24);
        m_fdc24TX = new CFDUC(24U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackTX);
    } else if (m_sampleRate == 24U) {
        LogMessage("Instantiating the dummy 24 kHz resampler");
        m_fdc24RX = new CFDCDummy(CModem::callbackRX24);
        m_fdc24TX = new CFDCDummy(CModem::callbackTX);
    } else {
        LogError("Invalid sample rate");
        return false;
    }

    if (m_sampleRate > 72U) {
        LogMessage("Instantiating the 72 kHz downsampler");
        m_fdc72RX = new CFDDC(72U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackRX72);
        m_fdc72TX = new CFDUC(72U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackTX);
    } else if (m_sampleRate == 72U) {
        LogMessage("Instantiating the dummy 72 kHz resampler");
        m_fdc72RX = new CFDCDummy(CModem::callbackRX72);
        m_fdc72TX = new CFDCDummy(CModem::callbackTX);
    } else {
        LogMessage("Instantiating the 72 kHz upsampler");
        m_fdc72RX = new CFDUC(72U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackRX72);
        m_fdc72TX = new CFDDC(72U, m_sampleRate, 1U, 12U, 11U, 0.5F, CModem::callbackTX);
    }

    m_format = MODEM_FORMAT(data[3U]);

    m_maxTXSamples = data[4U] + data[5U] * 256U;

    const char* proto = nullptr;

    switch (m_format) {
    case MODEM_FORMAT::BASEBAND: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * BB_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
            proto = "Baseband";
        }
        break;

    case MODEM_FORMAT::IQ_8: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * IQ8_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
            proto = "8-bit I/Q";
        }
        break;

    case MODEM_FORMAT::IQ_12: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * IQ12_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
            proto = "12-bit I/Q";
        }
        break;

    case MODEM_FORMAT::IQ_16: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * IQ16_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
            proto = "16-bit I/Q";
        }
        break;

    default:
        LogError("Unknown TX format - 0x%02X", m_format);
        return false;
    }

    LogMessage("Modem Information:");
    LogMessage("\tProtocol version: %u", data[0U]);
    LogMessage("\tCapabilities:");
    LogMessage("\t\tDuplex: %s", m_duplex ? "yes" : "no");
    LogMessage("\t\tTransmit: %s", m_hasTX ? "yes" : "no");
    LogMessage("\t\tReceive: %s", m_hasRX ? "yes" : "no");
    LogMessage("\tSample rate: %ukHz", m_sampleRate);
    LogMessage("\tFormat: %s", proto);
    LogMessage("\tMax TX samples: %u", m_maxTXSamples);
    LogMessage("\tFirmware version: \"%.*s\"", length - 6U, data + 6U);

    return true;
}

bool CModem::writeTransmitDataBB(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.dataSize() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if ((m_toModem.dataSize() > 0U) && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t marker  = 0x00U;

    int16_t* ptr = (int16_t*)m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.getData(&sample, 1U) && (n < m_maxTXSamples)) {
        *ptr++ = sample.iValue;

        if (sample.control != 0x00U) {
            offset = n;
            marker = sample.control;
        }

        n++;
    }

    m_spaceLeft -= n;

    uint16_t len = (n * BB_ELEMENT_LEN) + 10U;

    uint8_t buffer[10U];

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    buffer[7U] = 0x00U;

    // The unused RSSI and COS bytes
    buffer[8U] = 0x00U;
    buffer[9U] = 0x00U;

    m_modem.write(buffer, 10U);

    m_modem.write(m_txBuffer, len - 10U);

    if (m_trace) {
        dump("Write BB Data 1/2", buffer, 10U);
        dump("Write BB Data 2/2", m_txBuffer, len - 10U);
    } else {
        LogDebug("Sent Modem::TRANSMIT_DATA BB");
    }

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.hasData())
        m_transmitTimer.start();

    return true;
}

bool CModem::writeTransmitDataIQ8(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.dataSize() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if ((m_toModem.dataSize() > 0U) && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t marker  = 0x00U;

    int8_t* ptr = (int8_t*)m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.getData(&sample, 1U) && (n < m_maxTXSamples)) {
        *ptr++ = sample.iValue >> 12;
        *ptr++ = sample.qValue >> 12;

        if (sample.control != 0x00U) {
            offset = n;
            marker = sample.control;
        }

        n++;
    }

    m_spaceLeft -= n;

    uint16_t len = (n * IQ8_ELEMENT_LEN) + 8U;

    uint8_t buffer[8U];

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    buffer[7U] = 0x00U;

    m_modem.write(buffer, 8U);

    m_modem.write(m_txBuffer, len - 8U);

    if (m_trace) {
        dump("Write 8-bit IQ Data 1/2", buffer, 8U);
        dump("Write 8-bit IQ Data 2/2", m_txBuffer, len - 8U);
    } else {
        LogDebug("Sent Modem::TRANSMIT_DATA 8-bit IQ");
    }

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.hasData())
        m_transmitTimer.start();

    return true;
}

bool CModem::writeTransmitDataIQ12(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.dataSize() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if ((m_toModem.dataSize() > 0U) && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t  marker = 0x00U;

    uint8_t* p = m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.getData(&sample, 1U) && (n < m_maxTXSamples)) {
        // Convert to big-endian 12-bit I/Q format
        *p++  = (sample.iValue >> 8)  & 0xFFU;
        *p    = (sample.iValue >> 0)  & 0xF0U;
        *p++ |= (sample.qValue >> 12) & 0x0FU;
        *p++  = (sample.qValue >> 4)  & 0xFFU;

        if (sample.control != 0x00U) {
            offset = n;
            marker = sample.control;
        }

        n++;
    }

    m_spaceLeft -= n;

    uint16_t len = (n * IQ12_ELEMENT_LEN) + 8U;

    uint8_t buffer[8U];

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    buffer[7U] = 0x00U;

    m_modem.write(buffer, 8U);

    m_modem.write(m_txBuffer, len - 8U);

    if (m_trace) {
        dump("Write 12-bit IQ Data 1/2", buffer, 8U);
        dump("Write 12-bit IQ Data 2/2", m_txBuffer, len - 8U);
    } else {
        LogDebug("Sent Modem::TRANSMIT_DATA 12-bit IQ");
    }

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.hasData())
        m_transmitTimer.start();

    return true;
}

bool CModem::writeTransmitDataIQ16(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.dataSize() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if (m_toModem.hasData() && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t  marker = 0x00U;

    int16_t* ptr = (int16_t*)m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.getData(&sample, 1U) && (n < m_maxTXSamples)) {
        *ptr++ = sample.iValue;
        *ptr++ = sample.qValue;

        if (sample.control != 0x00U) {
            offset = n;
            marker = sample.control;
        }

        n++;
    }

    m_spaceLeft -= n;

    uint16_t len = (n * IQ16_ELEMENT_LEN) + 8U;

    uint8_t buffer[8U];

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    buffer[7U] = 0x00U;

    m_modem.write(buffer, 8U);

    m_modem.write(m_txBuffer, len - 8U);

    if (m_trace) {
        dump("Write 16-bit IQ Data 1/2", buffer, 8U);
        dump("Write 16-bit IQ Data 2/2", m_txBuffer, len - 8U);
    } else {
        LogDebug("Sent Modem::TRANSMIT_DATA 16-bit IQ");
    }

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.hasData())
        m_transmitTimer.start();

    return true;
}

bool CModem::processSampleFSK24BB(uint8_t marker, int16_t frequency, float32_t amplitude)
{
    IQSample<int16_t> sample;

    sample.control = marker;
    sample.iValue  = frequency;
    sample.qValue  = 0;

    // Send straight to the modem, no resampling is needed
    m_toModem.addData(&sample, 1U);

    return true;
}

bool CModem::processSampleFSK24IQ(uint8_t marker, int16_t frequency, float32_t amplitude)
{
    assert(m_fdc24TX != nullptr);

    m_phase += frequency * DEVIATION;

    float32_t phase = float32_t(m_phase) * M_PI;

    IQSample<float32_t> sample;

    sample.iValue  = amplitude * ::cos(phase);
    sample.qValue  = amplitude * ::sin(phase);
    sample.control = marker;

    m_fdc24TX->process(sample);

    return true;
}

bool CModem::processSamplePSK72IQ(uint8_t marker, int16_t phase, float32_t amplitude)
{
    assert(m_fdc72TX != nullptr);

    // Create I + Q

    IQSample<float32_t> sample;

    sample.iValue  = amplitude * ::arm_sin_q15(phase);
    sample.qValue  = amplitude * ::arm_cos_q15(phase);
    sample.control = marker;

    m_fdc72TX->process(sample);

    return true;
}

void CModem::processBBRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    uint16_t rssi = data[0U];           // LSB
    rssi |= (data[1U] & 0x7FU) << 8;    // MSB minus COS

    bool cos = (data[1U] & 0x80U) == 0x80U;

    uint16_t count = (length - sizeof(uint16_t)) / BB_ELEMENT_LEN;

    const int16_t* p = (const int16_t*)data;

    for (uint16_t i = 0U; i < count; i++) {
        q15_t sample = *p++;

        if (i == offset)
            io.read24FSK(marker, sample, cos, rssi);
        else
            io.read24FSK(MARK_NONE, sample, cos, rssi);
    }
}

void CModem::processIQ8RX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    assert(m_fdc24RX != nullptr);
    assert(m_fdc72RX != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    uint16_t count = length / IQ8_ELEMENT_LEN;

    const int8_t* p = (const int8_t*)data;

    for (uint16_t i = 0U; i < count; i++) {
        int16_t iData = (*p++) << 8;
        int16_t qData = (*p++) << 8;

        IQSample<float32_t> sample;
        sample.iValue = INT16_TO_FLOAT32(iData);
        sample.qValue = INT16_TO_FLOAT32(qData);
        sample.control = (i == offset) ? marker : 0x00U;

        m_fdc24RX->process(sample);
        m_fdc72RX->process(sample);
    }
}

void CModem::processIQ12RX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    assert(m_fdc24RX != nullptr);
    assert(m_fdc72RX != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    uint16_t count = length / IQ12_ELEMENT_LEN;

    const uint8_t* p = data;

    for (uint16_t i = 0U; i < count; i++) {
        int16_t iData, qData;

        // Convert from big-endian 12-bit I/Q format
        iData  = ((*p++) & 0xFFU) << 8;
        iData |= ((*p)   & 0xF0U) << 0;

        qData  = ((*p++) & 0x0FU) << 12;
        qData |= ((*p++) & 0xFFU) << 4;

        IQSample<float32_t> sample;

        sample.iValue = INT16_TO_FLOAT32(iData);
        sample.qValue = INT16_TO_FLOAT32(qData);
        sample.control = (i == offset) ? marker : 0x00U;

        m_fdc24RX->process(sample);
        m_fdc72RX->process(sample);
    }
}

void CModem::processIQ16RX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    assert(m_fdc24RX != nullptr);
    assert(m_fdc72RX != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    uint16_t count = length / IQ16_ELEMENT_LEN;

    const int16_t* p = (const int16_t*)data;

    for (uint16_t i = 0U; i < count; i++) {
        IQSample<float32_t> sample;

        sample.iValue = INT16_TO_FLOAT32(*p++);
        sample.qValue = INT16_TO_FLOAT32(*p++);
        sample.control = (i == offset) ? marker : 0x00U;

        m_fdc24RX->process(sample);
        m_fdc72RX->process(sample);
    }
}

void CModem::dump(const char* text, const uint8_t* data, uint16_t length) const
{
    assert(text != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    LogDebug("%s:", text);

    char line[100U];

    unsigned int offset = 0U;

    while (length > 0U) {
        ::sprintf(line, "%04X:  ", offset);

        unsigned int bytes = (length > 16U) ? 16U : length;

        for (unsigned i = 0U; i < bytes; i++)
            ::sprintf(line + ::strlen(line), "%02X ", data[offset + i]);

        for (unsigned int i = bytes; i < 16U; i++)
            ::strcat(line, "   ");

        ::strcat(line, "   *");

        for (unsigned i = 0U; i < bytes; i++) {
            uint8_t c = data[offset + i];

            if (::isprint(c))
                ::sprintf(line + ::strlen(line), "%c", c);
            else
                ::strcat(line, ".");
        }

        ::strcat(line, "*");

        LogDebug(line);

        offset += 16U;

        if (length >= 16U)
            length -= 16U;
        else
            length = 0U;
    }
}

void CModem::callbackTX(const IQSample<float32_t>& sample)
{
    assert(m_ptr != nullptr);

    IQSample<int16_t> sample2;

    sample2.iValue  = ::FLOAT32_TO_INT16(sample.iValue);
    sample2.qValue  = ::FLOAT32_TO_INT16(sample.qValue);
    sample2.control = sample.control;

    m_ptr->m_toModem.addData(&sample2, 1U);
}

void CModem::callbackRX24(const IQSample<float32_t>& sample)
{
    assert(m_ptr != nullptr);

    // Put main selectivity here

    bool cos = true;        // XXX

    float32_t rssi = ::sqrt(sample.iValue * sample.iValue + sample.qValue * sample.qValue);

    // Multiply by the conjugate of the last I/Q sample
    float32_t i = (sample.iValue * m_ptr->m_lastI24) + (sample.qValue * m_ptr->m_lastQ24);
    float32_t q = (sample.qValue * m_ptr->m_lastI24) - (sample.iValue * m_ptr->m_lastQ24);

    float32_t phase = ::atan2(q, i);

    // The base system is based on 12-bits of precision 
    q15_t frequency = ::FLOAT32_TO_Q15(phase) >> 4;

    io.read24FSK(sample.control, frequency, cos, uint16_t(rssi));

    m_ptr->m_lastI24 = sample.iValue;
    m_ptr->m_lastQ24 = sample.qValue;
}

void CModem::callbackRX72(const IQSample<float32_t>& sample)
{
    assert(m_ptr != nullptr);

    // Put main selectivity here

/*
    bool cos = true;        // XXX

    float32_t rssi = ::sqrt(sample.iValue * sample.iValue + sample.qValue * sample.qValue);

    // Multiply by the conjugate of the last I/Q sample
    float32_t i = (sample.iValue * m_ptr->m_lastI72) + (sample.qValue * m_ptr->m_lastQ72);
    float32_t q = (sample.qValue * m_ptr->m_lastI72) - (sample.iValue * m_ptr->m_lastQ72);

    float32_t phase = ::atan2(q, i);

    io.read72PSK(sample.control, phase, cos, uint16_t(rssi));

    m_ptr->m_lastI72 = sample.iValue;
    m_ptr->m_lastQ72 = sample.qValue;
*/
}
