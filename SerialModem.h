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
#include "RingBuffer.h"
#include "IQSample.h"
#include "Timer.h"
#include "FDUDC.h"

#include "arm_math.h"

#include <cstdint>

enum class SERIALMODEM_STATE {
	NONE,
	WAIT_VERSION,
	WAIT_FREQ_POWER,
	WAIT_START,
	WAIT_STOP,
	RUNNING
};

enum class SERIALMODEM_FORMAT : uint8_t {
	BASEBAND,
	IQ,
	NONE = 255U
};

class CSerialModem {
public:
	CSerialModem();
	~CSerialModem();

	void setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);

	bool hasDuplex() const;
	bool hasTX() const;
	bool hasRX() const;

	bool canTETRA() const;

	void start();

	void process();

	bool writeSample24(uint8_t marker, q15_t frequency, uint8_t amplitude = 255U);

	uint16_t getTXSpace() const;
	bool     isTX() const;

private:
	CUARTController   m_serial;
	SERIALMODEM_STATE m_state;
	uint8_t*          m_rxBuffer;
	uint16_t          m_rxPtr;
	uint16_t          m_rxLen;
	CTimer            m_timer;

	uint8_t           m_power;
	uint32_t          m_txFreq;
	uint32_t          m_rxFreq;
	uint32_t          m_pocsagFreq;

	bool               m_duplex;
	bool               m_hasTX;
	bool               m_hasRX;
	uint8_t            m_sampleRate;
	SERIALMODEM_FORMAT m_txFormat;
	SERIALMODEM_FORMAT m_rxFormat;
	uint16_t           m_maxTXSamples;

	IFDUDC*           m_fdudc24;
	IFDUDC*           m_fdudc72;

	CRingBuffer<IQSampleU16> m_toModem;
	CRingBuffer<IQSampleU16> m_fromModem;

	CRingBuffer<IQSampleF32> m_toModem24;
	CRingBuffer<IQSampleF32> m_fromModem24;

	CRingBuffer<IQSampleF32> m_toModem72;
	CRingBuffer<IQSampleF32> m_fromModem72;

	uint16_t          m_spaceLeft;
	bool              m_tx;

	uint32_t          m_phase;
	int16_t           m_lastPhase;
	float32_t         m_lastI24;
	float32_t         m_lastQ24;
	float32_t         m_lastI72;
	float32_t         m_lastQ72;

	static CSerialModem* m_ptr;

	void processMessage(uint8_t type, const uint8_t* data, uint16_t length);

	void writeGetVersion();
	void writeSetFreqPower();
	void writeStart();
	void writeStop();
	bool writeTransmitData(uint8_t marker, uint16_t offset, const uint8_t* buffer, uint16_t len);

	bool processBB24TX(uint8_t marker, int16_t frequency, float32_t amplitude);

	bool processIQ24TX(uint8_t marker, int16_t frequency, float32_t amplitude);
	bool processIQ72TX(uint8_t marker, int16_t phase, float32_t amplitude);

	void processBBRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);
	void processIQRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);

	void processIQ24RX(uint8_t marker, float32_t iValue, float32_t qValue);
	void processIQ72RX(uint8_t marker, float32_t iValue, float32_t qValue);

	void processVersion(const uint8_t* data, uint16_t length);

	void dump(const char* text, const uint8_t* data, uint16_t length) const;

	static void callback72(float32_t& iValue, float32_t& qValue);
	static void callback24(float32_t& iValue, float32_t& qValue);
};

#endif
