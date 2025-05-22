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

#include "arm_math.h"

#include <cstdint>

enum class SERIALMODEM_STATE {
	NONE,
	WAIT_VERSION,
	WAIT_FREQ_POWER,
	WAIT_START,
	WAIT_STOP,
	WAIT_TRANSMIT_START,
	WAIT_TRANSMIT_STOP,
	RUNNING
};

class CSerialModem {
public:
	CSerialModem();
	~CSerialModem();

	void setParams(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);

	bool hasDuplex() const;
	bool hasTX() const;
	bool hasRX() const;

	bool canPSK() const;

	void start();

	void process();

	bool writeFrequencyAndAmplitudeSample24(uint8_t marker, int16_t frequency, uint8_t amplitude = 255U);
	bool writePhaseAndAmplitudeSample72(uint8_t marker, int16_t phase, uint8_t amplitude = 255U);

	void writeTransmitStart();
	void writeTransmitStop();

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
	uint16_t          m_txOffset;
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
	uint16_t          m_maxTXSamples;

	uint16_t          m_spaceLeft;
	bool              m_tx;

	uint32_t          m_phase;
	int16_t           m_lastPhase;
	q15_t             m_lastI;
	q15_t             m_lastQ;

	arm_fir_instance_q15 m_upsampleFilter;
	q15_t                m_upsampleState[48U];      // NoTaps + BlockSize - 1, 39 + 10 - 1 plus some spare
	arm_fir_instance_q15 m_downsampleFilter;
	q15_t                m_downsampleState[48U];    // NoTaps + BlockSize - 1, 39 + 10 - 1 plus some spare

	void processMessage(uint8_t type, const uint8_t* data, uint16_t length);

	void writeGetVersion();
	void writeSetFreqPower();
	void writeStart();
	void writeStop();
	bool writeTransmitData(uint8_t marker, uint16_t offset, const uint8_t* buffer, uint16_t len);

	uint8_t fsk24ToType0(int16_t frequency, uint8_t amplitude);
	uint8_t fsk24ToType1(int16_t frequency, uint8_t amplitude);
	uint8_t fsk24ToType2(int16_t frequency, uint8_t amplitude);

	uint8_t psk72ToType1(int16_t phase, uint8_t amplitude);
	uint8_t psk72ToType2(int16_t phase, uint8_t amplitude);

	void processType0(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);
	void processType1(uint8_t marker, uint16_t offset, const uint8_t* data, uint16_t length);

	q15_t normaliseQ15(q15_t val1, q15_t val2) const;
	q31_t normaliseQ31(q31_t val1, q31_t val2) const;

	void upsample24Kto72K(q15_t in, q15_t* out);
	bool downsample72Kto24K(q15_t in, q15_t& out);

	void processVersion(const uint8_t* data, uint16_t length);

	void dump(const char* text, const uint8_t* data, uint16_t length) const;
};

#endif
