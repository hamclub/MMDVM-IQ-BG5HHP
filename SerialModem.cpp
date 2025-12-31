/*
 *   Copyright (C) 2025 by Jonathan Naylor G4KLX
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

#include "SerialModem.h"
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
const uint8_t TYPE_STATUS                   = 0x04U;
const uint8_t TYPE_TRANSMIT_DATA            = 0x05U;
const uint8_t TYPE_RECEIVE_DATA             = 0x06U;
const uint8_t TYPE_DEBUG_MESSAGE            = 0xF0U;
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

const uint8_t  STOP_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_STOP};
const uint16_t STOP_MESSAGE_LEN = sizeof(STOP_MESSAGE) / sizeof(uint8_t);

const uint16_t BB_ELEMENT_LEN = sizeof(uint16_t);
const uint16_t IQ_ELEMENT_LEN = sizeof(int16_t) + sizeof(int16_t);

CSerialModem* CSerialModem::m_ptr = nullptr;


CSerialModem::CSerialModem() :
m_serial(),
m_state(SERIALMODEM_STATE::NONE),
m_txBuffer(nullptr),
m_rxBuffer(nullptr),
m_rxPtr(0U),
m_rxLen(0U),
m_messageTimer(1000U, 0U, 100U),
m_watchdogTimer(1000U, 1U),
m_transmitTimer(1000U, 0U, 200U),
m_power(0U),
m_txFreq(0U),
m_rxFreq(0U),
m_pocsagFreq(0U),
m_duplex(false),
m_hasTX(false),
m_hasRX(false),
m_sampleRate(0U),
m_txFormat(SERIALMODEM_FORMAT::NONE),
m_rxFormat(SERIALMODEM_FORMAT::NONE),
m_maxTXSamples(0U),
m_fddc24RX(nullptr),
m_fduc24TX(nullptr),
m_fddc72RX(nullptr),
m_fduc72TX(nullptr),
m_toModem(5000U),
m_spaceLeft(0U),
m_tx(false),
m_phase(0U),
m_lastPhase(0),
m_lastI24(0.0F),
m_lastQ24(0.0F),
m_lastI72(0.0F),
m_lastQ72(0.0F),
m_stopwatch()
{
    m_ptr = this;
}

CSerialModem::~CSerialModem()
{
    delete[] m_txBuffer;
    delete[] m_rxBuffer;

    delete m_fddc24RX;
    delete m_fduc24TX;
    delete m_fddc72RX;
    delete m_fduc72TX;
}

void CSerialModem::setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
    m_power      = power;
    m_txFreq     = txFreq;
    m_rxFreq     = rxFreq;
    m_pocsagFreq = pocsagFreq;

    writeSetFreqPower();
}

bool CSerialModem::hasDuplex() const
{
    return m_duplex;
}

bool CSerialModem::hasTX() const
{
    return m_hasTX;
}

bool CSerialModem::hasRX() const
{
    return m_hasRX;
}

bool CSerialModem::canTETRA() const
{
    return (m_txFormat == SERIALMODEM_FORMAT::IQ) && (m_rxFormat == SERIALMODEM_FORMAT::IQ);
}

bool CSerialModem::start(const std::string& port, unsigned int speed)
{
    bool ret = m_serial.open(port.c_str(), speed);
    if (!ret)
        return false;

    m_spaceLeft = 0U;

    m_state = SERIALMODEM_STATE::WAIT_VERSION;

    m_stopwatch.start();
    m_messageTimer.start();

    return true;
}

// Called from the main loop
void CSerialModem::process()
{
    // Handle the receive data
    uint8_t c;
	while (m_serial.read(&c, 1U) > 0U) {
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
                processMessage(m_rxBuffer[3U], m_rxBuffer + 4U, m_rxLen - 4U);
                m_rxPtr = 0U;
                m_rxLen = 0U;
            }
        }
    }

    // Is this correct?
    unsigned int ms = m_stopwatch.elapsed();
    m_messageTimer.clock(ms);
    m_watchdogTimer.clock(ms);
    m_transmitTimer.clock(ms);
    m_stopwatch.start();

    if (m_messageTimer.isRunning() && m_messageTimer.hasExpired()) {
        switch (m_state) {
        case SERIALMODEM_STATE::NONE:
            m_messageTimer.stop();
            break;
        case SERIALMODEM_STATE::WAIT_VERSION:
            writeGetVersion();
            m_messageTimer.start();
            break;
        case SERIALMODEM_STATE::WAIT_FREQ_POWER:
            writeSetFreqPower();
            m_messageTimer.start();
            break;
        case SERIALMODEM_STATE::WAIT_START:
            writeStart();
            m_messageTimer.start();
            break;
        case SERIALMODEM_STATE::WAIT_STOP:
            writeStop();
            m_messageTimer.start();
            break;
        default:	// SMS_RUNNING
            m_messageTimer.stop();
            break;
        }
    }

    // Handle the transmit data
    switch (m_txFormat) {
    case SERIALMODEM_FORMAT::BASEBAND:
        writeTransmitDataBB(false);
        break;
    case SERIALMODEM_FORMAT::IQ:
        writeTransmitDataIQ(false);
        break;
    default:
        break;
    }

    if (m_transmitTimer.isRunning() && m_transmitTimer.hasExpired()) {
        ::printf("The transmit timer has expired\n");
        // Handle the remaining transmit data
        switch (m_txFormat) {
        case SERIALMODEM_FORMAT::BASEBAND:
            writeTransmitDataBB(true);
            break;
        case SERIALMODEM_FORMAT::IQ:
            writeTransmitDataIQ(true);
            break;
        default:
            break;
        }
    }

    if (m_watchdogTimer.isRunning() && m_watchdogTimer.hasExpired()) {
        ::printf("The watchdog timer has expired\n");
        // Actually not sure what to do here
    }
}

// For all modes bar TETRA and P25 phase 2
bool CSerialModem::writeSampleFSK24(uint8_t marker, q15_t frequency, uint8_t amplitude)
{
    if (m_state != SERIALMODEM_STATE::RUNNING)
        return false;

    if (m_sampleRate < 24U)
        return false;

    switch (m_txFormat) {
    case SERIALMODEM_FORMAT::BASEBAND:
        return processSampleFSK24BB(marker, frequency, amplitude);

    case SERIALMODEM_FORMAT::IQ:
        return processSampleFSK24IQ(marker, frequency, amplitude);

    default:
        ::fprintf(stderr, "Unknown TX format for FSK24\n");
        return false;
    }
}

uint16_t CSerialModem::getTXSpace() const
{
    return m_spaceLeft;
}

bool CSerialModem::isTX() const
{
    return m_tx;
}

void CSerialModem::processMessage(uint8_t type, const uint8_t* data, uint16_t length)
{
    switch (type) {
    case TYPE_GET_VERSION:
        if (m_state == SERIALMODEM_STATE::WAIT_VERSION) {
            bool ret = processVersion(data, length);
            if (!ret)
                throw;

            m_state = SERIALMODEM_STATE::NONE;
            m_messageTimer.start();
        }
        break;
    case TYPE_DEBUG_MESSAGE:
        ::printf("DEBUG: %.*s\n", length, data);
        break;
    case TYPE_ACK:
        if (m_state == SERIALMODEM_STATE::WAIT_FREQ_POWER) {
            writeStart();
            m_messageTimer.start();
        } else if (m_state == SERIALMODEM_STATE::WAIT_START) {
            ::printf("Modem has been started\n");
            m_state = SERIALMODEM_STATE::RUNNING;
            m_messageTimer.stop();
        }
        break;
    case TYPE_NAK:
        switch (data[1U]) {
        case ERR_UNKNOWN_MESSAGE_TYPE:
            ::printf("Unknown message type for message type 0x%02X\n", data[0U]);
            break;
        case ERR_MALFORMED_MESSAGE:
            ::printf("Malformed message received for message type 0x%02X\n", data[0U]);
            break;
        case ERR_TRANSMIT_DATA_BEFORE_START:
            ::printf("Modem not started for message type 0x%02X\n", data[0U]);
            break;
        case ERR_TRANSMIT_BUFFER_OVERFLOW:
            ::printf("Transmit buffer overflow for message type 0x%02X\n", data[0U]);
            break;
        case ERR_INVALID_FREQUENCY:
            ::printf("Invalid frequency for the hardware for message type 0x%02X\n", data[0U]);
            break;
        case ERR_TRANSMIT_DATA_ERROR:
            ::printf("Transmit data error for message type 0x%02X\n", data[0U]);
            break;
        case ERR_OUT_OF_MEMORY:
            ::printf("Out of memory in the modem for message type 0x%02X\n", data[0U]);
            break;
        default:
            ::printf("Unknown error type - 0x%02X for message type 0x%02X\n", data[1U], data[0U]);
            break;
        }
        ::fflush(stdout);
        throw;
    case TYPE_STATUS:
        m_spaceLeft = data[0U] + (data[1U] * 256U);
        m_tx = (data[2U] & STATUS_TX_ON) == STATUS_TX_ON;
        m_watchdogTimer.start();
        break;
    case TYPE_RECEIVE_DATA:
        if (m_state == SERIALMODEM_STATE::RUNNING) {
            uint8_t  marker = data[0U];
            uint16_t offset = data[1U] + (data[2U] * 256U);

            switch (m_rxFormat) {
            case SERIALMODEM_FORMAT::BASEBAND:
                processBBRX(marker, offset, data + 4U, length - 4U);
                break;
            case SERIALMODEM_FORMAT::IQ:
                processIQRX(marker, offset, data + 4U, length - 4U);
                break;
            default:
                break;
            }
        }
        break;
    default:
        ::fprintf(stderr, "Unknown message type - 0x%02X\n", type);
        break;
    }
}

void CSerialModem::writeGetVersion()
{
    m_serial.write(GET_VERSION_MESSAGE, GET_VERSION_MESSAGE_LEN);

    m_state = SERIALMODEM_STATE::WAIT_VERSION;
}

void CSerialModem::writeSetFreqPower()
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

    m_serial.write(buffer, 13U);

    m_state = SERIALMODEM_STATE::WAIT_FREQ_POWER;
}

void CSerialModem::writeStart()
{
    uint8_t buffer[6U];

    buffer[0U] = FRAME_START;
    buffer[1U] = 0x06U;
    buffer[2U] = 0x00U;
    buffer[3U] = TYPE_START;

    buffer[4U] = MAX_RX_SAMPLES % 256U;     // LSB
    buffer[5U] = MAX_RX_SAMPLES / 256U;     // MSB

    m_serial.write(buffer, 6U);

    m_state = SERIALMODEM_STATE::WAIT_START;

    m_watchdogTimer.start();
}

void CSerialModem::writeStop()
{
    m_serial.write(STOP_MESSAGE, STOP_MESSAGE_LEN);

    m_state = SERIALMODEM_STATE::WAIT_STOP;

    m_toModem.reset();

    m_watchdogTimer.stop();
    m_transmitTimer.stop();
}

bool CSerialModem::processVersion(const uint8_t* data, uint16_t length)
{
    m_duplex = (data[1U] & CAP_DUPLEX)       == CAP_DUPLEX;
    m_hasTX  = (data[1U] & CAP_HAS_TRANSMIT) == CAP_HAS_TRANSMIT;
    m_hasRX  = (data[1U] & CAP_HAS_RECEIVE)  == CAP_HAS_RECEIVE;

    m_sampleRate = data[2U];

    if (m_sampleRate > 24U) {
        ::printf("Instantiating the 24 kHz downsampler\n");
        m_fddc24RX = new CFDDC(24U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fddc24RX->setCallback(CSerialModem::callbackRX24);

        m_fduc24TX = new CFDUC(24U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fduc24TX->setCallback(CSerialModem::callbackTX);
    } else if (m_sampleRate == 24U) {
        ::printf("Instantiating the dummy 24 kHz resampler\n");
        m_fddc24RX = new CFDUDCDummy;
        m_fddc24RX->setCallback(CSerialModem::callbackRX24);

        m_fduc24TX = new CFDUDCDummy;
        m_fduc24TX->setCallback(CSerialModem::callbackTX);
    } else {
        ::fprintf(stderr, "Invalid sample rate\n");
        return false;
    }

    if (m_sampleRate > 72U) {
        ::printf("Instantiating the 72 kHz downsampler\n");
        m_fddc72RX = new CFDDC(72U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fddc72RX->setCallback(CSerialModem::callbackRX72);

        m_fduc72TX = new CFDUC(72U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fduc72TX->setCallback(CSerialModem::callbackTX);
    } else if (m_sampleRate == 72U) {
        ::printf("Instantiating the dummy 72 kHz resampler\n");
        m_fddc72RX = new CFDUDCDummy;
        m_fddc72RX->setCallback(CSerialModem::callbackRX72);

        m_fduc72TX = new CFDUDCDummy;
        m_fduc72TX->setCallback(CSerialModem::callbackTX);
    } else {
        ::printf("Instantiating the 72 kHz upsampler\n");
        m_fddc72RX = new CFDUC(72U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fddc72RX->setCallback(CSerialModem::callbackRX72);

        m_fduc72TX = new CFDDC(72U, m_sampleRate, 1U, 12U, 1U, 12U, 11U, 0.5F);
        m_fduc72TX->setCallback(CSerialModem::callbackTX);
    }

    m_txFormat = SERIALMODEM_FORMAT(data[3U]);
    m_rxFormat = SERIALMODEM_FORMAT(data[4U]);

    m_maxTXSamples = data[5U] + data[6U] * 256U;

    switch (m_txFormat) {
    case SERIALMODEM_FORMAT::BASEBAND: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * BB_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
        }
        break;

    case SERIALMODEM_FORMAT::IQ: {
            delete[] m_txBuffer;
            uint16_t bytes = m_maxTXSamples * IQ_ELEMENT_LEN;
            m_txBuffer = new uint8_t[bytes + 20U];
        }
        break;

    default:
        ::fprintf(stderr, "Unknown TX format - %u\n", m_txFormat);
        return false;
    }

    switch (m_rxFormat) {
    case SERIALMODEM_FORMAT::BASEBAND: {
            delete[] m_rxBuffer;
            uint16_t bytes = MAX_RX_SAMPLES * BB_ELEMENT_LEN;
            m_rxBuffer = new uint8_t[bytes + 20U];
        }
        break;

    case SERIALMODEM_FORMAT::IQ: {
            delete[] m_rxBuffer;
            uint16_t bytes = MAX_RX_SAMPLES * IQ_ELEMENT_LEN;
            m_rxBuffer = new uint8_t[bytes + 20U];
        }
        break;

    default:
        ::fprintf(stderr, "Unknown RX format - %u\n", m_rxFormat);
        return false;
    }

    ::printf("Modem version:\n");
    ::printf("\tProtocol version: %u\n", data[0U]);
    ::printf("\tCapabilities:\n");
    ::printf("\t\tDuplex: %s\n", m_duplex ? "yes" : "no");
    ::printf("\t\tTransmit: %s\n", m_hasTX ? "yes" : "no");
    ::printf("\t\tReceive: %s\n", m_hasRX ? "yes" : "no");
    ::printf("\t\tSample rate: %ukHz\n", m_sampleRate);
    ::printf("\tFormats:\n");
    ::printf("\t\tTransmit: %u\n", m_txFormat);
    ::printf("\t\tReceive: %u\n", m_rxFormat);
    ::printf("\tMax TX samples: %u\n", m_maxTXSamples);
    ::printf("\tVersion: \"%.*s\"\n", length - 7U, data + 7U);

    return true;
}

bool CSerialModem::writeTransmitDataBB(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.getData() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if ((m_toModem.getData() > 0U) && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t marker  = 0x00U;

    int16_t* ptr = (int16_t*)m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.get(sample) && (n < m_maxTXSamples)) {
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

    m_serial.write(buffer, 10U);

    m_serial.write(m_txBuffer, len);

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.getData() > 0U)
        m_transmitTimer.start();

    return true;
}

bool CSerialModem::writeTransmitDataIQ(bool flush)
{
    if (!flush) {
        // If not flushing the data, only transmit the data if we have a buffer full
        if (m_toModem.getData() < m_maxTXSamples)
            return false;
    } else {
        // If flushing the data, only transmit when the modem has space
        if ((m_toModem.getData() > 0U) && (m_spaceLeft > 0U)) {
            m_transmitTimer.start();
            return false;
        }
    }

    uint16_t offset = 0U;
    uint8_t marker  = 0x00U;

    int16_t* ptr = (int16_t*)m_txBuffer;
    uint16_t n = 0U;
    IQSample<int16_t> sample;
    while (m_toModem.get(sample) && (n < m_maxTXSamples)) {
        *ptr++ = sample.iValue;
        *ptr++ = sample.qValue;

        if (sample.control != 0x00U) {
            offset = n;
            marker = sample.control;
        }

        n++;
    }

    m_spaceLeft -= n;

    uint16_t len = (n * IQ_ELEMENT_LEN) + 8U;

    uint8_t buffer[8U];

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    buffer[7U] = 0x00U;

    m_serial.write(buffer, 8U);

    m_serial.write(m_txBuffer, len);

    if (!flush)
        m_transmitTimer.start();
    else if (m_toModem.getData() > 0U)
        m_transmitTimer.start();

    return true;
}

bool CSerialModem::processSampleFSK24BB(uint8_t marker, int16_t frequency, float32_t amplitude)
{
    IQSample<int16_t> sample;

    sample.control = marker;
    sample.iValue  = frequency;
    sample.qValue  = 0;

    // Send straight to the modem, no resampling is needed
    m_toModem.put(sample);

    return true;
}

bool CSerialModem::processSampleFSK24IQ(uint8_t marker, int16_t frequency, float32_t amplitude)
{
    assert(m_fduc24TX != nullptr);

    m_phase += frequency * DEVIATION;

    float32_t phase = float32_t(m_phase) * M_PI;

    IQSample<float32_t> sample;

    sample.iValue  = amplitude * ::cos(phase);
    sample.qValue  = amplitude * ::sin(phase);
    sample.control = marker;

    m_fduc24TX->process(sample);

    return true;
}

bool CSerialModem::processSamplePSK72IQ(uint8_t marker, int16_t phase, float32_t amplitude)
{
    assert(m_fduc72TX != nullptr);

    // Create I + Q

    IQSample<float32_t> sample;

    sample.iValue  = amplitude * ::arm_sin_q15(phase);
    sample.qValue  = amplitude * ::arm_cos_q15(phase);
    sample.control = marker;

    m_fduc72TX->process(sample);

    return true;
}

void CSerialModem::processBBRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
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

void CSerialModem::processIQRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    assert(m_fddc24RX != nullptr);
    assert(m_fddc72RX != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    uint16_t count = length / IQ_ELEMENT_LEN;

    const int16_t* p = (const int16_t*)data;

    for (uint16_t i = 0U; i < count; i++) {
        IQSample<float32_t> sample;

        sample.iValue = INT16_TO_FLOAT32(*p++);
        sample.qValue = INT16_TO_FLOAT32(*p++);
        sample.control = (i == offset) ? marker : 0x00U;

        m_fddc24RX->process(sample);
        m_fddc72RX->process(sample);
    }
}

void CSerialModem::dump(const char* text, const uint8_t* data, uint16_t length) const
{
    assert(text != nullptr);
    assert(data != nullptr);
    assert(length > 0U);

    ::printf("%s:\n", text);

    unsigned int offset = 0U;

    while (length > 0U) {
        ::printf("%04X:  ", offset);

        unsigned int bytes = (length > 16U) ? 16U : length;

        for (unsigned i = 0U; i < bytes; i++)
            ::printf("%02X ", data[offset + i]);

        for (unsigned int i = bytes; i < 16U; i++)
            ::printf("   ");

        ::printf("   *");

        for (unsigned i = 0U; i < bytes; i++) {
            uint8_t c = data[offset + i];

            if (::isprint(c))
                ::printf("%c", c);
            else
                ::printf(".");
        }

        ::printf("*\n");

        offset += 16U;

        if (length >= 16U)
            length -= 16U;
        else
            length = 0U;
    }
}

void CSerialModem::callbackTX(const IQSample<float32_t>& sample)
{
    assert(m_ptr != nullptr);

    IQSample<int16_t> sample2;

    sample2.iValue  = ::FLOAT32_TO_INT16(sample.iValue);
    sample2.qValue  = ::FLOAT32_TO_INT16(sample.qValue);
    sample2.control = sample.control;

    m_ptr->m_toModem.put(sample2);
}

void CSerialModem::callbackRX24(const IQSample<float32_t>& sample)
{
    assert(m_ptr != nullptr);

    // Put main selectivity here

    bool cos = true;        // XXX

    float32_t rssi = ::sqrt(sample.iValue * sample.iValue + sample.qValue * sample.qValue);

    // Multiply by the conjugate of the last I/Q sample
    float32_t i = (sample.iValue * m_ptr->m_lastI24) + (sample.qValue * m_ptr->m_lastQ24);
    float32_t q = (sample.qValue * m_ptr->m_lastI24) - (sample.iValue * m_ptr->m_lastQ24);

    float32_t phase = ::atan2(q, i);

    io.read24FSK(sample.control, ::FLOAT32_TO_Q15(phase), cos, uint16_t(rssi));

    m_ptr->m_lastI24 = sample.iValue;
    m_ptr->m_lastQ24 = sample.qValue;
}

void CSerialModem::callbackRX72(const IQSample<float32_t>& sample)
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
