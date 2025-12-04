/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025 by Jonathan Naylor G4KLX
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

#if !defined(IO_H)
#define  IO_H

#include "Globals.h"

#include "RingBuffer.h"

class TSample {
public:
	TSample(q15_t sample, uint8_t control, bool cos, uint16_t rssi) :
	m_sample(sample),
	m_control(control),
	m_cos(cos),
	m_rssi(rssi)
	{
	}

	TSample() :
	m_sample(0),
	m_control(0U),
	m_cos(false),
	m_rssi(0U)
	{
	}

	q15_t    m_sample;
	uint8_t  m_control;
	bool     m_cos;
	uint16_t m_rssi;
};

class CIO {
public:
  CIO();

  void start();

  void process();

  void read24FSK(uint8_t marker, const q15_t frequency, bool cos, uint16_t rssi);
  void read72PSK(uint8_t marker, const q15_t phase, uint16_t rssi);

  void write24FSK(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control = NULL);
  void write72PSK(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control = NULL);

  uint16_t getSpace() const;

  void setDecode(bool dcd);
  void setADCDetection(bool detect);
  void setMode(MMDVM_STATE state);
  
  void setParameters(bool rxInvert, bool txInvert, bool pttInvert, uint8_t rxLevel, uint8_t cwIdTXLevel, uint8_t dstarTXLevel, uint8_t dmrTXLevel, uint8_t ysfTXLevel, uint8_t p25TXLevel, uint8_t nxdnTXLevel, uint8_t pocsagTXLevel, uint8_t fmTXLevel, int16_t txDCOffset, int16_t rxDCOffset, bool useCOSAsLockout);
  void setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);

  bool hasLockout() const;

  void resetWatchdog();
  uint32_t getWatchdog();
  
  uint8_t getCPU() const;

  void getUDID(uint8_t* buffer);

  void selfTest();

private:
  bool                  m_started;
  CRingBuffer<TSample>  m_rxFSKBuffer;

#if defined(USE_DCBLOCKER)
  arm_biquad_casd_df1_inst_q31 m_dcFilter;
  q31_t                        m_dcState[4];
#endif

#if defined(MODE_DSTAR)
  arm_fir_instance_q15 m_gaussianFilter;
  q15_t                m_gaussianState[40U];      // NoTaps + BlockSize - 1, 12 + 20 - 1 plus some spare
#endif

#if defined(MODE_DMR)
  arm_fir_instance_q15 m_rrc02Filter1;
  q15_t                m_rrc02State1[70U];         // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
#endif

#if defined(MODE_YSF)
  arm_fir_instance_q15 m_rrc02Filter2;
  q15_t                m_rrc02State2[70U];         // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
#endif

#if defined(MODE_P25)
  arm_fir_instance_q15 m_boxcar5Filter;
  q15_t                m_boxcar5State[30U];        // NoTaps + BlockSize - 1,  6 + 20 - 1 plus some spare
#endif

#if defined(MODE_NXDN)
#if defined(USE_NXDN_BOXCAR)
  arm_fir_instance_q15 m_boxcar10Filter;
  q15_t                m_boxcar10State[40U];      // NoTaps + BlockSize - 1, 10 + 20 - 1 plus some spare
#else
  arm_fir_instance_q15 m_nxdnFilter;
  arm_fir_instance_q15 m_nxdnISincFilter;
  q15_t                m_nxdnState[110U];         // NoTaps + BlockSize - 1, 82 + 20 - 1 plus some spare
  q15_t                m_nxdnISincState[60U];     // NoTaps + BlockSize - 1, 32 + 20 - 1 plus some spare
#endif
#endif

  bool                 m_pttInvert;
  q15_t                m_rxLevel;
  q15_t                m_cwIdTXLevel;
  q15_t                m_dstarTXLevel;
  q15_t                m_dmrTXLevel;
  q15_t                m_ysfTXLevel;
  q15_t                m_p25TXLevel;
  q15_t                m_nxdnTXLevel;
  q15_t                m_pocsagTXLevel;
  q15_t                m_fmTXLevel;

  uint16_t             m_rxDCOffset;
  uint16_t             m_txDCOffset;

  bool                 m_useCOSAsLockout;

  uint32_t             m_ledCount;
  bool                 m_ledValue;

  bool                 m_detect;

  volatile uint32_t    m_watchdog;

  bool                 m_lockout;

  // Hardware specific routines
  void initInt();
  void startInt();

  bool getCOSInt();

  void setLEDInt(bool on);
  void setPTTInt(bool on);
  void setCOSInt(bool on);

  void delayInt(unsigned int dly);
};

#endif
