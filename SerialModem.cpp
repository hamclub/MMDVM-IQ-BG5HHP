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

const uint8_t FRAME_START = 0xC0U;

const uint8_t TYPE_GET_VERSION              = 0x00U;
const uint8_t TYPE_VERSION_RESPONSE         = 0x01U;
const uint8_t TYPE_SET_FREQUENCY_AND_POWER  = 0x02U;
const uint8_t TYPE_START                    = 0x03U;
const uint8_t TYPE_STOP                     = 0x04U;
const uint8_t TYPE_STATUS                   = 0x05U;
const uint8_t TYPE_TRANSMIT_DATA            = 0x06U;
const uint8_t TYPE_TRANSMIT_DATA_WITH_RESET = 0x07U;
const uint8_t TYPE_RECEIVE_DATA             = 0x08U;
const uint8_t TYPE_GET_ACK                  = 0xFEU;
const uint8_t TYPE_GET_NAK                  = 0xFFU;

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
m_len(0U)
{
}

void CSerialModem::start()
{
	m_serial.open("\\\\.\\COM4", 921600U);
}

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

	switch (m_state) {
	case SMS_NONE:
		m_serial.write(GET_VERSION_MESSAGE, GET_VERSION_MESSAGE_LEN);
		m_state = SMS_WAIT_VERSION;
		break;
	case SMS_WAIT_VERSION:
		break;
	case SMS_WAIT_START:
		break;
	default:	// SMS_RUNNING
		break;
	}
}
