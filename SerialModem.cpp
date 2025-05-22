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

#include <cstdio>
#include <cmath>

#if !defined(M_PI)
const float M_PI = 3.141592654F;
#endif

const uint8_t FRAME_START = 0xC0U;

const uint8_t TYPE_GET_VERSION              = 0x00U;
const uint8_t TYPE_VERSION_RESPONSE         = 0x01U;
const uint8_t TYPE_SET_FREQUENCY_AND_POWER  = 0x02U;
const uint8_t TYPE_START                    = 0x03U;
const uint8_t TYPE_STOP                     = 0x04U;
const uint8_t TYPE_TRANSMIT_START           = 0x05U;
const uint8_t TYPE_TRANSMIT_STOP            = 0x06U;
const uint8_t TYPE_STATUS                   = 0x07U;
const uint8_t TYPE_TRANSMIT_DATA            = 0x08U;
const uint8_t TYPE_RECEIVE_DATA             = 0x09U;
const uint8_t TYPE_DEBUG_MESSAGE            = 0xF0U;
const uint8_t TYPE_ACK                      = 0xFEU;
const uint8_t TYPE_NAK                      = 0xFFU;

const uint8_t CAP_HAS_DUPLEX                = 0x01U;
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

const uint32_t SAMPLE_RATE_72K = 72000U;
const uint32_t SAMPLE_RATE_24K = 24000U;

const uint8_t SR_24K_TO_72K = SAMPLE_RATE_72K / SAMPLE_RATE_24K;

const int32_t DEVIATION = 500000;               // TODO

const uint8_t TYPE0_ELEMENT_LEN = 2U;
const uint8_t TYPE1_ELEMENT_LEN = 4U;
const uint8_t TYPE2_ELEMENT_LEN = 2U;
const uint8_t TYPE3_ELEMENT_LEN = 3U;

const uint8_t  GET_VERSION_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_GET_VERSION};
const uint16_t GET_VERSION_MESSAGE_LEN = sizeof(GET_VERSION_MESSAGE) / sizeof(uint8_t);

const uint8_t  STOP_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_STOP};
const uint16_t STOP_MESSAGE_LEN = sizeof(STOP_MESSAGE) / sizeof(uint8_t);

const uint8_t  START_TRANSMIT_MESSAGE[] = { FRAME_START, 0x04U, 0x00U, TYPE_TRANSMIT_START };
const uint16_t START_TRANSMIT_MESSAGE_LEN = sizeof(START_TRANSMIT_MESSAGE) / sizeof(uint8_t);

const uint8_t  STOP_TRANSMIT_MESSAGE[] = { FRAME_START, 0x04U, 0x00U, TYPE_TRANSMIT_STOP };
const uint16_t STOP_TRANSMIT_MESSAGE_LEN = sizeof(STOP_TRANSMIT_MESSAGE) / sizeof(uint8_t);

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 72000 Hz

* 0 Hz - 9000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.8553535932915217 dB

* 11050 Hz - 36000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.6697367094349 dB
*/
static q15_t SAMPLE_FILTER[] = {
    314,   339, 231, -157, -740, -1265, -1420, -1029, -209,  630,  977,   530,  -550,
  -1626, -1858, -65, 1933, 5148,  7801,  8827,  7801, 5148, 1933, -657, -1858, -1626,
   -550,   530, 977,  630, -209, -1029, -1420, -1265, -740, -157,  231,   339,   314};
const uint16_t SAMPLE_FILTER_LEN = 39U;

CSerialModem::CSerialModem() :
m_serial(),
m_state(SERIALMODEM_STATE::NONE),
m_rxBuffer(NULL),
m_txBuffer(NULL),
m_rxPtr(0U),
m_rxLen(0U),
m_txLen(0U),
m_txMarker(0x00U),
m_txOffset(0U),
m_timer(1000U, 0U, 100U),
m_power(0U),
m_txFreq(0U),
m_rxFreq(0U),
m_pocsagFreq(0U),
m_hasDuplex(false),
m_hasTX(false),
m_hasRX(false),
m_txFormat(0xFFU),
m_rxFormat(0xFFU),
m_maxTXSamples(0U),
m_spaceLeft(0U),
m_tx(false),
m_phase(0U),
m_lastPhase(0),
m_lastI(0),
m_lastQ(0),
m_upsampleFilter(),
m_upsampleState(),
m_downsampleFilter(),
m_downsampleState()
{
    ::memset(m_upsampleState, 0x00U, 48U * sizeof(q15_t));
    m_upsampleFilter.numTaps = SAMPLE_FILTER_LEN;
    m_upsampleFilter.pState  = m_upsampleState;
    m_upsampleFilter.pCoeffs = SAMPLE_FILTER;

    ::memset(m_downsampleState, 0x00U, 48U * sizeof(q15_t));
    m_downsampleFilter.numTaps = SAMPLE_FILTER_LEN;
    m_downsampleFilter.pState  = m_downsampleState;
    m_downsampleFilter.pCoeffs = SAMPLE_FILTER;
}

CSerialModem::~CSerialModem()
{
    delete[] m_rxBuffer;
    delete[] m_txBuffer;
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
    return m_hasDuplex;
}

bool CSerialModem::hasTX() const
{
    return m_hasTX;
}

bool CSerialModem::hasRX() const
{
    return m_hasRX;
}

bool CSerialModem::canPSK() const
{
    // This may need updating when new hardware becomes available
    return ((m_txFormat == FORMAT_FREQUENCY_AND_AMPLITUDE) || (m_txFormat == FORMAT_IQ)) && (m_rxFormat == FORMAT_IQ);
}

void CSerialModem::start()
{
    m_serial.open("\\\\.\\COM8", 921600U);

    m_spaceLeft = 0U;

    m_state = SERIALMODEM_STATE::WAIT_VERSION;

    m_timer.start();
}

// Called every 10ms from the main loop
void CSerialModem::process()
{
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

    m_timer.clock(10U);
    if (m_timer.isRunning() && m_timer.hasExpired()) {
        switch (m_state) {
        case SERIALMODEM_STATE::NONE:
            m_timer.stop();
            break;
        case SERIALMODEM_STATE::WAIT_VERSION:
            writeGetVersion();
            m_timer.start();
            break;
        case SERIALMODEM_STATE::WAIT_FREQ_POWER:
            writeSetFreqPower();
            m_timer.start();
            break;
        case SERIALMODEM_STATE::WAIT_START:
            writeStart();
            m_timer.start();
            break;
        case SERIALMODEM_STATE::WAIT_STOP:
            writeStop();
            m_timer.start();
            break;
        case SERIALMODEM_STATE::WAIT_TRANSMIT_START:
            writeTransmitStart();
            m_timer.start();
            break;
        case SERIALMODEM_STATE::WAIT_TRANSMIT_STOP:
            writeTransmitStop();
            m_timer.start();
            break;
        default:	// SMS_RUNNING
            m_timer.stop();
            break;
        }
    }
}

// For all modes bar TETRA and P25 phase 2
bool CSerialModem::writeFrequencyAndAmplitudeSample24(uint8_t marker, int16_t frequency, uint8_t amplitude)
{
    if (m_state != SERIALMODEM_STATE::RUNNING)
        return false;

    if (marker != MARK_NONE) {
        m_txOffset = m_txLen;
        m_txMarker = marker;
    }

    uint16_t txLen = 0U;

    switch (m_txFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        if (m_txLen == 0U) {
            // The unused COS and RSSI bytes
            m_txBuffer[m_txLen++] = 0x00U;
            m_txBuffer[m_txLen++] = 0x00U;
        }
        txLen = 1U;
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        txLen = SR_24K_TO_72K;
        break;
    case FORMAT_IQ:
        txLen = SR_24K_TO_72K;
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        break;
    default:
        break;
    }

    if (txLen == 0U) {
        ::fprintf(stderr, "Unknown TX format for FSK24\n");
        return false;
    }

    txLen += m_txLen;

    if ((txLen >= m_spaceLeft) || (txLen >= m_maxTXSamples)) {
        writeTransmitData(m_txMarker, m_txOffset, m_txBuffer, m_txLen);
        m_txMarker = MARK_NONE;
        m_txOffset = 0U;
        m_txLen = 0U;
    }

    switch (m_txFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        m_txLen += fsk24ToType0(frequency, amplitude);
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        m_txLen += fsk24ToType1(frequency, amplitude);
        break;
    case FORMAT_IQ:
        m_txLen += fsk24ToType2(frequency, amplitude);
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        return false;
    default:
        return false;
    }

    return true;
}

// For TETRA and P25 phase 2
bool CSerialModem::writePhaseAndAmplitudeSample72(uint8_t marker, int16_t phase, uint8_t amplitude)
{
    if (m_state != SERIALMODEM_STATE::RUNNING)
        return false;

    if (marker != MARK_NONE) {
        m_txOffset = m_txLen;
        m_txMarker = marker;
    }

    uint16_t txLen = 0U;

    switch (m_txFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        txLen = 1U;
        break;
    case FORMAT_IQ:
        txLen = 1U;
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        break;
    default:
        break;
    }

    txLen += m_txLen;

    if (txLen == 0U) {
        ::fprintf(stderr, "Unknown TX format for PSK72\n");
        return false;
    }

    if ((txLen >= m_spaceLeft) || (txLen >= m_maxTXSamples)) {
        writeTransmitData(m_txMarker, m_txOffset, m_txBuffer, m_txLen);
        m_txMarker = MARK_NONE;
        m_txOffset = 0U;
        m_txLen = 0U;
    }

    switch (m_txFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        return false;
    case FORMAT_IQ:
        m_txLen += psk72ToType1(phase, amplitude);
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        m_txLen += psk72ToType2(phase, amplitude);
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        return false;
    default:
        return false;
    }

    return true;
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
    case TYPE_VERSION_RESPONSE:
        if (m_state == SERIALMODEM_STATE::WAIT_VERSION) {
            processVersion(data, length);
            m_state = SERIALMODEM_STATE::NONE;
            m_timer.start();
        }
        break;
    case TYPE_DEBUG_MESSAGE:
        ::printf("DEBUG: %.*s\n", length, data);
        break;
    case TYPE_ACK:
        if (m_state == SERIALMODEM_STATE::WAIT_FREQ_POWER) {
            writeStart();
            m_timer.start();
        } else if (m_state == SERIALMODEM_STATE::WAIT_START) {
            ::printf("Modem has been started\n");
            m_state = SERIALMODEM_STATE::RUNNING;
            m_timer.stop();
        } else if (m_state == SERIALMODEM_STATE::WAIT_TRANSMIT_START) {
            m_state = SERIALMODEM_STATE::RUNNING;
            m_timer.stop();
        } else if (m_state == SERIALMODEM_STATE::WAIT_TRANSMIT_STOP) {
            m_state = SERIALMODEM_STATE::RUNNING;
            m_timer.stop();
        }
        break;
    case TYPE_NAK:
        switch (data[0U]) {
        case ERR_UNKNOWN_MESSAGE_TYPE:
            ::printf("Unknown message type\n");
            break;
        case ERR_MALFORMED_MESSAGE:
            ::printf("Malformed message received\n");
            break;
        case ERR_TRANSMIT_DATA_BEFORE_START:
            ::printf("Modem not started\n");
            break;
        case ERR_TRANSMIT_BUFFER_OVERFLOW:
            ::printf("Transmit buffer overflow\n");
            break;
        case ERR_INVALID_FREQUENCY:
            ::printf("Invalid frequency for the hardware\n");
            break;
        case ERR_TRANSMIT_DATA_ERROR:
            ::printf("Transmit data error\n");
            break;
        case ERR_OUT_OF_MEMORY:
            ::printf("Out of memory in the modem\n");
            break;
        default:
            ::printf("Unknown error type - 0x%02X\n", data[0U]);
            break;
        }
        ::fflush(stdout);
        throw;
    case TYPE_STATUS:
        m_spaceLeft = data[0U] + (data[1U] * 256U);
        m_tx = (data[2U] & STATUS_TX_ON) == STATUS_TX_ON;
        break;
    case TYPE_RECEIVE_DATA:
        if (m_state == SERIALMODEM_STATE::RUNNING) {
            uint8_t  marker = data[0U];
            uint16_t offset = data[1U] + (data[2U] * 256U);

            switch (m_rxFormat) {
            case FORMAT_BASEBAND_AND_RSSI:
                processType0(marker, offset, data + 3U, length - 3U);
                break;
            case FORMAT_IQ:
                processType1(marker, offset, data + 3U, length - 3U);
                break;
            case FORMAT_FREQUENCY_AND_AMPLITUDE:
                break;
            case FORMAT_PHASE_AND_AMPLITUDE:
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
}

void CSerialModem::writeStop()
{
    m_serial.write(STOP_MESSAGE, STOP_MESSAGE_LEN);

    m_state = SERIALMODEM_STATE::WAIT_STOP;
}

void CSerialModem::writeTransmitStart()
{
    m_serial.write(START_TRANSMIT_MESSAGE, START_TRANSMIT_MESSAGE_LEN);

    m_state = SERIALMODEM_STATE::WAIT_TRANSMIT_START;
}

void CSerialModem::writeTransmitStop()
{
    m_serial.write(STOP_TRANSMIT_MESSAGE, STOP_TRANSMIT_MESSAGE_LEN);

    m_state = SERIALMODEM_STATE::WAIT_TRANSMIT_STOP;
}

void CSerialModem::processVersion(const uint8_t* data, uint16_t length)
{
    m_hasDuplex = (data[1U] & CAP_HAS_DUPLEX) == CAP_HAS_DUPLEX;
    m_hasTX     = (data[1U] & CAP_HAS_TRANSMIT) == CAP_HAS_TRANSMIT;
    m_hasRX     = (data[1U] & CAP_HAS_RECEIVE) == CAP_HAS_RECEIVE;

    m_txFormat = data[2U];
    m_rxFormat = data[3U];

    uint8_t txSizeMultiplier = 0U;
    uint8_t rxSizeMultiplier = 0U;

    switch (m_txFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        txSizeMultiplier = sizeof(int16_t);
        break;
    case FORMAT_IQ:
        txSizeMultiplier = sizeof(int16_t) + sizeof(int16_t);
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        txSizeMultiplier = sizeof(uint8_t) + sizeof(int8_t);
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        txSizeMultiplier = sizeof(uint8_t) + sizeof(int16_t);
        break;
    default:
        ::fprintf(stderr, "Unknown TX format - %u\n", m_txFormat);
        throw;
    }

    switch (m_rxFormat) {
    case FORMAT_BASEBAND_AND_RSSI:
        rxSizeMultiplier = sizeof(int16_t);
        break;
    case FORMAT_IQ:
        rxSizeMultiplier = sizeof(int16_t) + sizeof(int16_t);
        break;
    case FORMAT_FREQUENCY_AND_AMPLITUDE:
        rxSizeMultiplier = sizeof(uint8_t) + sizeof(int8_t);
        break;
    case FORMAT_PHASE_AND_AMPLITUDE:
        rxSizeMultiplier = sizeof(uint8_t) + sizeof(int16_t);
        break;
    default:
        ::fprintf(stderr, "Unknown RX format - %u\n", m_rxFormat);
        throw;
    }

    m_maxTXSamples = data[4U] + data[5U] * 256U;

    uint16_t txBytes   = m_maxTXSamples * txSizeMultiplier;
    uint16_t rxBytes   = MAX_RX_SAMPLES * rxSizeMultiplier;

    ::printf("Modem version:\n");
    ::printf("\tProtocol version: %u\n", data[0U]);
    ::printf("\tCapabilities:\n");
    ::printf("\t\tDuplex: %s\n", m_hasDuplex ? "yes" : "no");
    ::printf("\t\tTransmit: %s\n", m_hasTX ? "yes" : "no");
    ::printf("\t\tReceive: %s\n", m_hasRX ? "yes" : "no");
    ::printf("\tFormats:\n");
    ::printf("\t\tTransmit: %u\n", m_txFormat);
    ::printf("\t\tReceive: %u\n", m_rxFormat);
    ::printf("\tMax TX samples: %u\n", m_maxTXSamples);
    ::printf("\tVersion: \"%.*s\"\n", length - 6U, data + 6U);
    ::printf("\tTX size multiplier: %u\n", txSizeMultiplier);
    ::printf("\tRX size multiplier: %u\n", rxSizeMultiplier);

    delete[] m_txBuffer;
    delete[] m_rxBuffer;

    m_rxBuffer = new uint8_t[rxBytes + 20U];
    m_txBuffer = new uint8_t[txBytes + 20U];

    // This may need updating when new hardware becomes available
    if ((m_txFormat != FORMAT_BASEBAND_AND_RSSI) && (m_txFormat != FORMAT_IQ) && (m_txFormat != FORMAT_FREQUENCY_AND_AMPLITUDE)) {
        ::printf("Invalid transmit format - %u\n", data[2U]);
        ::fflush(stdout);
        throw;
    }

    // This may need updating when new hardware becomes available
    if ((m_rxFormat != FORMAT_BASEBAND_AND_RSSI) && (m_rxFormat != FORMAT_IQ)) {
        ::printf("Invalid receive format - %u\n", data[3U]);
        ::fflush(stdout);
        throw;
    }
}

bool CSerialModem::writeTransmitData(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t len)
{
    uint8_t buffer[10U];

    if (m_spaceLeft < len)
        return false;

    m_spaceLeft -= len;

    len += 7U;

    buffer[0U] = FRAME_START;
    buffer[1U] = len % 256U;
    buffer[2U] = len / 256U;
    buffer[3U] = TYPE_TRANSMIT_DATA;

    buffer[4U] = marker;

    buffer[5U] = offset % 256U;
    buffer[6U] = offset / 256U;

    m_serial.write(buffer, 7U);

    m_serial.write(data, len);

    return true;
}

// XXX What about RSSI bytes?
uint8_t CSerialModem::fsk24ToType0(int16_t frequency, uint8_t amplitude)
{
    m_txBuffer[m_txLen++] = uint8_t(frequency % 256);       // LSB
    m_txBuffer[m_txLen++] = uint8_t(frequency / 256);       // MSB

    return 1U;
}

uint8_t CSerialModem::fsk24ToType1(int16_t frequency, uint8_t amplitude)
{
    // Upsample to 72 kHz and then create I + Q

    q15_t out[SR_24K_TO_72K];
    upsample24Kto72K(frequency, out);

    for (uint8_t j = 0U; j < SR_24K_TO_72K; j++) {
        m_phase += out[j] * DEVIATION;

        // 2 * PI * t = 29826 in Q31 for 72 kHz sample rate
        q31_t phaseQ31 = normaliseQ31(m_phase, 29826);

        q31_t q31 = amplitude * ::arm_sin_q31(phaseQ31);
        q31_t i31 = amplitude * ::arm_cos_q31(phaseQ31);

        q15_t q15 = q31 >> 16;
        q15_t i15 = i31 >> 16;

        m_txBuffer[m_txLen++] = uint8_t(i15 % 256);     // LSB
        m_txBuffer[m_txLen++] = uint8_t(i15 / 256);     // MSB

        m_txBuffer[m_txLen++] = uint8_t(q15 % 256);     // LSB
        m_txBuffer[m_txLen++] = uint8_t(q15 / 256);     // MSB
    }

    return SR_24K_TO_72K;
}

uint8_t CSerialModem::fsk24ToType2(int16_t frequency, uint8_t amplitude)
{
    // Upsample to 72 kHz and then create Frequency + Amplitude

    q15_t out[SR_24K_TO_72K];
    upsample24Kto72K(frequency, out);

    for (uint8_t j = 0U; j < SR_24K_TO_72K; j++) {
        m_txBuffer[m_txLen++] = uint8_t(out[j] / 24);
        m_txBuffer[m_txLen++] = amplitude;
    }

    return SR_24K_TO_72K;
}

uint8_t CSerialModem::psk72ToType1(int16_t phase, uint8_t amplitude)
{
    // Create I + Q

    q15_t q = amplitude * ::arm_sin_q15(phase);
    q15_t i = amplitude * ::arm_cos_q15(phase);

    m_txBuffer[m_txLen++] = uint8_t(i % 256);       // LSB
    m_txBuffer[m_txLen++] = uint8_t(i / 256);       // MSB

    m_txBuffer[m_txLen++] = uint8_t(q % 256);       // LSB
    m_txBuffer[m_txLen++] = uint8_t(q / 256);       // MSB

    return 1U;
}

uint8_t CSerialModem::psk72ToType2(int16_t phase, uint8_t amplitude)
{
    // Create Frequency + Amplitude

    if (m_lastPhase != phase) {
        int16_t freq = (phase / 360) * SAMPLE_RATE_72K;

        m_txBuffer[m_txLen++] = uint8_t(freq / 24);
        m_txBuffer[m_txLen++] = amplitude;

        m_lastPhase = phase;
    } else {
        m_txBuffer[m_txLen++] = 0U;
        m_txBuffer[m_txLen++] = amplitude;
    }

    return 1U;
}

void CSerialModem::processType0(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    uint16_t rssi = data[0U];           // LSB
    rssi |= (data[1U] & 0x7FU) << 8;    // MSB minus COS

    bool cos = (data[1U] & 0x80U) == 0x80U;

    uint16_t count = (length - sizeof(uint16_t)) / TYPE0_ELEMENT_LEN;

    uint16_t n = sizeof(uint16_t);

    for (uint16_t i = 0U; n < count; i++) {
        q15_t sample  = int16_t(data[n++]);     // LSB
        sample |= int16_t(data[n++]) << 8;      // MSB

        if (i == offset)
            io.read24FSK(marker, sample, cos, rssi);
        else
            io.read24FSK(MARK_NONE, sample, cos, rssi);
    }
}

void CSerialModem::processType1(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length)
{
    uint16_t count = length / (2 * sizeof(int16_t));

    bool cos = true;        // XXX
    uint16_t rssi = 0;

    uint16_t n = 0U;
    for (uint16_t j = 0U; j < count; j++) {
        q15_t i15 = int16_t(data[n++]);         // LSB
        i15 |= int16_t(data[n++]) << 8;         // MSB

        q15_t q15 = int16_t(data[n++]);         // LSB
        q15 |= int16_t(data[n++]) << 8;         // MSB

        q31_t out = 0;
        ::arm_sqrt_q31(i15 * i15 + q15 * q15, &out);
        rssi += out;

        // Multiply by the conjugate of the last I/Q sample
        q31_t i = (i15 * m_lastI) + (q15 * m_lastQ);
        q31_t q = (q15 * m_lastI) - (i15 * m_lastQ);

        q15_t phase = ::arm_atan_q31(q, i) >> 16;

        q15_t freq = 0;
        bool ret = downsample72Kto24K(phase, freq);
        if (ret) {
            if (j == count)
                io.read24FSK(marker, freq, cos, (rssi / 3U) >> 16);
            else
                io.read24FSK(MARK_NONE, freq, cos, (rssi / 3U) >> 16);
            rssi = 0U;
        }

        if (j == count)
            io.read72PSK(marker, phase, rssi);
        else
            io.read72PSK(MARK_NONE, phase, rssi);

        m_lastI = i15;
        m_lastQ = q15;
    }
}

void CSerialModem::upsample24Kto72K(q15_t in, q15_t* out)
{
    q15_t temp[SR_24K_TO_72K];
    temp[0U] = in;
    temp[1U] = 0;
    temp[2U] = 0;

    ::arm_fir_fast_q15(&m_upsampleFilter, temp, out, SR_24K_TO_72K);
}

bool CSerialModem::downsample72Kto24K(q15_t in, q15_t& out)
{
    static uint8_t n = 0U;
    static q15_t tIn[SR_24K_TO_72K];

    tIn[n++] = in;
    if (n < 3U)
        return false;

    q15_t tOut[SR_24K_TO_72K];
    ::arm_fir_fast_q15(&m_downsampleFilter, tIn, tOut, SR_24K_TO_72K);

    out = tOut[0U];
    n = 0U;

    return true;
}

q15_t CSerialModem::normaliseQ15(q15_t val1, q15_t val2) const
{
    q31_t val = val1 * val2;

    while (val >= INT16_MAX)
        val -= INT16_MAX;

    while (val <= INT16_MIN)
        val += INT16_MIN;

    return q15_t(val);
}

q31_t CSerialModem::normaliseQ31(q31_t val1, q31_t val2) const
{
    q63_t val = val1 * val2;

    while (val >= INT32_MAX)
        val -= INT32_MAX;

    while (val <= INT32_MIN)
        val += INT32_MIN;

    return q31_t(val);
}

void CSerialModem::dump(const char* text, const uint8_t* data, uint16_t length) const
{
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
