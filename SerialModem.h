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

	void setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);

	bool hasDuplex() const;
	bool hasTX() const;
	bool hasRX() const;

	bool canPSK() const;

	void start();

	void process();

	bool writeFrequencyAndAmplitudeSample(uint8_t marker, int16_t frequency, uint8_t amplitude = 255U);
	bool writePhaseAndAmplitudeSample(uint8_t marker, int16_t phase, uint8_t amplitude = 255U);

	uint16_t getTXSpace() const;
	bool     isTX() const;

private:
	CUARTController   m_serial;
	SERIALMODEM_STATE m_state;
	uint8_t*          m_rxBuffer;
	uint8_t*          m_txBuffer;
	uint16_t          m_rxPtr;
	uint16_t          m_rxLen;
	uint16_t          m_txLen;
	uint8_t           m_txMarker;
	CTimer            m_timer;

	uint8_t           m_power;
	uint32_t          m_txFreq;
	uint32_t          m_rxFreq;
	uint32_t          m_pocsagFreq;

	bool              m_hasDuplex;
	bool              m_hasTX;
	bool              m_hasRX;
	uint8_t           m_txFormat;
	uint8_t           m_rxFormat;
	uint16_t          m_maxSize;

	uint8_t           m_sizeMultiplier;

	uint16_t          m_spaceLeft;
	bool              m_tx;

	void processMessage(uint8_t type, const uint8_t* data, uint16_t length);

	void writeGetVersion();
	void writeSetFreqPower();
	void writeStart();
	bool writeTransmitData(uint8_t marker, const uint8_t* buffer, uint16_t len);

	void processVersion(const uint8_t* data, uint16_t length);

	void dump(const char* text, const uint8_t* data, uint16_t length) const;
};

#endif
