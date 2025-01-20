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

#if !defined(SERIALMODEM_H)
#define  SERIALMODEM_H

#include "UARTController.h"
#include "Timer.h"

#include <cstdint>

const uint16_t BUFFER_LENGTH_SERIAL = 8000U;

enum SERIALMODEM_STATE {
	SMS_NONE,
	SMS_WAIT_VERSION,
	SMS_WAIT_FREQ_POWER,
	SMS_WAIT_START,
	SMS_RUNNING
};

class CSerialModem {
public:
	CSerialModem();

	void start();

	void process();

private:
	CUARTController   m_serial;
	SERIALMODEM_STATE m_state;
	uint8_t           m_buffer[BUFFER_LENGTH_SERIAL];
	uint16_t          m_ptr;
	uint16_t          m_len;
	CTimer            m_timer;

	uint8_t           m_power;
	uint32_t          m_rxFreq;
	uint32_t          m_txFreq;

	void processMessage(uint8_t type, const uint8_t* data, uint16_t length);

	void writeGetVersion();
	void writeSetFreqPower();
	void writeStart();

	void processVersion(const uint8_t* data, uint16_t length);

	void dump(const char* text, const uint8_t* data, uint16_t length) const;
};

#endif
