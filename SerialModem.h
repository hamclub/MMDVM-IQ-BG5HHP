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

#if !defined(SERIALMODEM_H)
#define  SERIALMODEM_H

#include "UARTController.h"
#include "RingBuffer.h"
#include "StopWatch.h"
#include "IQSample.h"
#include "Timer.h"
#include "FDC.h"

#include "arm_math.h"

#include <string>

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
	COMP_IQ,
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

	bool start(const std::string& port, unsigned int speed, bool trace);

	void process();

	bool writeSampleFSK24(uint8_t marker, q15_t frequency, uint8_t amplitude = 255U);

	uint16_t getTXSpace() const;
	bool     isTX() const;

private:
	CUARTController    m_serial;
	SERIALMODEM_STATE  m_state;
	uint8_t*           m_txBuffer;
	uint8_t*           m_rxBuffer;
	uint16_t           m_rxPtr;
	uint16_t           m_rxLen;
	CTimer             m_statusTimer;
	CTimer             m_messageTimer;
	CTimer             m_watchdogTimer;
	CTimer             m_transmitTimer;

	uint8_t            m_power;
	uint32_t           m_txFreq;
	uint32_t           m_rxFreq;
	uint32_t           m_pocsagFreq;

	bool               m_duplex;
	bool               m_hasTX;
	bool               m_hasRX;
	uint8_t            m_sampleRate;
	SERIALMODEM_FORMAT m_txFormat;
	SERIALMODEM_FORMAT m_rxFormat;
	uint16_t           m_maxTXSamples;

	IFDC*              m_fdc24RX;
	IFDC*              m_fdc24TX;
	IFDC*              m_fdc72RX;
	IFDC*              m_fdc72TX;

	CRingBuffer<IQSample<int16_t>> m_toModem;

	uint16_t           m_spaceLeft;
	bool               m_tx;

	uint32_t           m_phase;
	int16_t            m_lastPhase;
	float32_t          m_lastI24;
	float32_t          m_lastQ24;
	float32_t          m_lastI72;
	float32_t          m_lastQ72;

	CStopWatch         m_stopwatch;

	bool               m_trace;

	static CSerialModem* m_ptr;

	void processMessage(uint8_t type, const uint8_t* data, uint16_t length);

	void writeGetVersion();
	void writeSetFreqPower();
	void writeGetStatus();
	void writeStart();
	void writeStop();

	bool writeTransmitDataBB(bool flush);
	bool writeTransmitDataIQ(bool flush);
	bool writeTransmitDataCompIQ(bool flush);

	bool processSampleFSK24BB(uint8_t marker, int16_t frequency, float32_t amplitude);
	bool processSampleFSK24IQ(uint8_t marker, int16_t frequency, float32_t amplitude);

	bool processSamplePSK72IQ(uint8_t marker, int16_t phase, float32_t amplitude);

	void processBBRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);
	void processIQRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);
	void processCompIQRX(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);

	bool processVersion(const uint8_t* data, uint16_t length);

	void dump(const char* text, const uint8_t* data, uint16_t length) const;

	static void callbackTX(const IQSample<float32_t>& sample);
	static void callbackRX24(const IQSample<float32_t>& sample);
	static void callbackRX72(const IQSample<float32_t>& sample);
};

#endif
