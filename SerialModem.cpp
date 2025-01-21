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

const uint8_t FRAME_START = 0xC0U;

const uint8_t TYPE_GET_VERSION              = 0x00U;
const uint8_t TYPE_VERSION_RESPONSE         = 0x01U;
const uint8_t TYPE_SET_FREQUENCY_AND_POWER  = 0x02U;
const uint8_t TYPE_START                    = 0x03U;
const uint8_t TYPE_STOP                     = 0x04U;
const uint8_t TYPE_STATUS                   = 0x05U;
const uint8_t TYPE_TRANSMIT_DATA            = 0x06U;
const uint8_t TYPE_RECEIVE_DATA             = 0x07U;
const uint8_t TYPE_ACK                      = 0xFEU;
const uint8_t TYPE_NAK                      = 0xFFU;

const uint8_t CAP_HAS_DUPLEX                = 0x01U;
const uint8_t CAP_HAS_TRANSMIT              = 0x02U;
const uint8_t CAP_HAS_RECEIVE               = 0x04U;

const uint8_t ERR_UNKNOWN_MESSAGE_TYPE       = 0x00U;
const uint8_t ERR_MALFORMED_MESSAGE          = 0x01U;
const uint8_t ERR_TRANSMIT_DATA_BEFORE_START = 0x02U;
const uint8_t ERR_TRANSMIT_BUFFER_OVERFLOW   = 0x03U;
const uint8_t ERR_INVALID_FREQUENCY          = 0x04U;

const uint8_t  GET_VERSION_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_GET_VERSION};
const uint16_t GET_VERSION_MESSAGE_LEN = sizeof(GET_VERSION_MESSAGE) / sizeof(uint8_t);

const uint8_t  START_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_START};
const uint16_t START_MESSAGE_LEN = sizeof(START_MESSAGE) / sizeof(uint8_t);

const uint8_t  STOP_MESSAGE[]   = {FRAME_START, 0x04U, 0x00U, TYPE_STOP};
const uint16_t STOP_MESSAGE_LEN = sizeof(STOP_MESSAGE) / sizeof(uint8_t);

CSerialModem::CSerialModem() :
m_serial(),
m_state(SMS_NONE),
m_buffer(),
m_ptr(0U),
m_len(0U),
m_timer(1000U, 0U, 100U),
m_power(0U),
m_txFreq(0U),
m_rxFreq(0U),
m_hasDuplex(false),
m_hasTX(false),
m_hasRX(false),
m_txFormat(0xFFU),
m_rxFormat(0xFFU),
m_maxSize(0U),
m_spaceLeft(0U)
{
}

void CSerialModem::setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq)
{
    m_power  = power;
    m_txFreq = txFreq;
    m_rxFreq = rxFreq;
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

    m_state = SMS_WAIT_VERSION;

    m_timer.start();
}

// Called every 10ms from the main loop
void CSerialModem::process()
{
    uint8_t c;
	while (m_serial.read(&c, 1U) > 0U) {
        if (m_ptr == 0U) {
            if (c == FRAME_START) {
                // Handle the frame start correctly
                m_buffer[0U] = c;
                m_ptr = 1U;
                m_len = 0U;
            } else {
                m_ptr = 0U;
                m_len = 0U;
            }
        } else if (m_ptr == 1U) {
            // Handle the frame length, 1/2
            m_len = c;
            m_buffer[m_ptr] = c;
            m_ptr = 2U;
        } else if (m_ptr == 2U) {
            // Handle the frame length, 2/2
            m_len += uint16_t(c) * 256U;
            m_buffer[m_ptr] = c;
            m_ptr = 3U;
        } else {
            // Any other bytes are added to the buffer
            m_buffer[m_ptr] = c;
            m_ptr++;

            // The full packet has been received, process it
            if (m_ptr == m_len) {
                processMessage(m_buffer[3U], m_buffer + 4U, m_len - 4U);
                m_ptr = 0U;
                m_len = 0U;
            }
        }
    }

    m_timer.clock(10U);
    if (m_timer.isRunning() && m_timer.hasExpired()) {
        switch (m_state) {
        case SMS_NONE:
            m_timer.stop();
            break;
        case SMS_WAIT_VERSION:
            writeGetVersion();
            m_timer.start();
            break;
        case SMS_WAIT_FREQ_POWER:
            writeSetFreqPower();
            m_timer.start();
            break;
        case SMS_WAIT_START:
            writeStart();
            m_timer.start();
            break;
        default:	// SMS_RUNNING
            m_timer.stop();
            break;
        }
    }
}

void CSerialModem::processMessage(uint8_t type, const uint8_t* data, uint16_t length)
{
    switch (type) {
    case TYPE_VERSION_RESPONSE:
        if (m_state == SMS_WAIT_VERSION) {
            processVersion(data, length);
            writeSetFreqPower();
            m_state = SMS_WAIT_FREQ_POWER;
            m_timer.start();
        }
        break;
    case TYPE_ACK:
        if (m_state == SMS_WAIT_FREQ_POWER) {
            writeStart();
            m_state = SMS_WAIT_START;
            m_timer.start();
        } else if (m_state == SMS_WAIT_START) {
            ::printf("Modem has been started\n");
            m_state = SMS_RUNNING;
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
        default:
            ::printf("Unknown error type - 0x%02X\n", data[0U]);
            break;
        }
        ::fflush(stdout);
        throw;
    case TYPE_STATUS:
        m_spaceLeft = data[0U] + (data[1U] * 256U);
        break;
    case TYPE_RECEIVE_DATA:
        if (m_state == SMS_RUNNING) {
            switch (m_rxFormat) {
            case FORMAT_BASEBAND_AND_RSSI:
                break;
            case FORMAT_IQ:
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
}

void CSerialModem::writeSetFreqPower()
{
    uint8_t buffer[13U];

    buffer[0U] = FRAME_START;
    buffer[1U] = 0x0DU;
    buffer[2U] = 0x00U;
    buffer[3U] = TYPE_SET_FREQUENCY_AND_POWER;

    buffer[4U] = m_power;

    buffer[5U] = (m_rxFreq >> 0) & 0xFFU;
    buffer[6U] = (m_rxFreq >> 8) & 0xFFU;
    buffer[7U] = (m_rxFreq >> 16) & 0xFFU;
    buffer[8U] = (m_rxFreq >> 24) & 0xFFU;

    buffer[9U]  = (m_txFreq >> 0) & 0xFFU;
    buffer[10U] = (m_txFreq >> 8) & 0xFFU;
    buffer[11U] = (m_txFreq >> 16) & 0xFFU;
    buffer[12U] = (m_txFreq >> 24) & 0xFFU;

    m_serial.write(buffer, 13U);
}

void CSerialModem::writeStart()
{
    m_serial.write(START_MESSAGE, START_MESSAGE_LEN);
}

void CSerialModem::processVersion(const uint8_t* data, uint16_t length)
{
    m_hasDuplex = (data[1U] & CAP_HAS_DUPLEX) == CAP_HAS_DUPLEX;
    m_hasTX     = (data[1U] & CAP_HAS_TRANSMIT) == CAP_HAS_TRANSMIT;
    m_hasRX     = (data[1U] & CAP_HAS_RECEIVE) == CAP_HAS_RECEIVE;

    m_txFormat = data[2U];
    m_rxFormat = data[3U];

    m_maxSize = data[4U] + data[5U] * 256U;

    ::printf("Modem version:\n");
    ::printf("\tProtocol version: %u\n", data[0U]);
    ::printf("\tCapabilities:\n");
    ::printf("\t\tDuplex: %s\n", m_hasDuplex ? "yes" : "no");
    ::printf("\t\tTransmit: %s\n", m_hasTX ? "yes" : "no");
    ::printf("\t\tReceive: %s\n", m_hasRX ? "yes" : "no");
    ::printf("\tFormats:\n");
    ::printf("\t\tTransmit: %u\n", m_txFormat);
    ::printf("\t\tReceive: %u\n", m_rxFormat);
    ::printf("\tMax size: %u\n", m_maxSize);
    ::printf("\tVersion: \"%.*s\"\n", length - 6U, data + 6U);

    // This may need updating when new hardware becomes available
    if ((m_txFormat != FORMAT_BASEBAND_AND_RSSI) && (m_txFormat != FORMAT_FREQUENCY_AND_AMPLITUDE)) {
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
