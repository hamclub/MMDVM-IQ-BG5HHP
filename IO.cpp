/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2026 by Adrian Musceac YO8RZZ
 *   Copyright (C) 2026 by Shawn Chain BG5HHP
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

#include "Globals.h"
#include "Config.h"
#include "IO.h"

#include "IOSoapy.h"
#include "SDRMulti.h"

#include <cstdio>
#include <cassert>

#if !defined(M_PI)
#define M_PI    3.141592654
#endif

#if defined(USE_DCBLOCKER)
// Generated using [b, a] = butter(1, 0.001) in MATLAB
static q31_t   DC_FILTER[] = {3367972, 0, 3367972, 0, 2140747704, 0}; // {b0, 0, b1, b2, -a1, -a2}
const uint32_t DC_FILTER_STAGES = 1U; // One Biquad stage
#endif

#if defined(MODE_DMR) || defined(MODE_YSF)
// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
static q15_t RRC_0_2_FILTER[] = {401, 104, -340, -731, -847, -553, 112, 909, 1472, 1450, 683, -675, -2144, -3040, -2706, -770, 2667, 6995,
                                 11237, 14331, 15464, 14331, 11237, 6995, 2667, -770, -2706, -3040, -2144, -675, 683, 1450, 1472, 909, 112,
                                 -553, -847, -731, -340, 104, 401, 0};
const uint16_t RRC_0_2_FILTER_LEN = 42U;
#endif

#if defined(MODE_NXDN)
#if defined(USE_NXDN_BOXCAR)
// One symbol boxcar filter
static q15_t   BOXCAR10_FILTER[] = {6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000};
const uint16_t BOXCAR10_FILTER_LEN = 10U;
#else
// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
static q15_t NXDN_0_2_FILTER[] = {284, 198, 73, -78, -240, -393, -517, -590, -599, -533, -391, -181, 79, 364, 643, 880, 1041, 1097, 1026, 819,
                                  483, 39, -477, -1016, -1516, -1915, -2150, -2164, -1914, -1375, -545, 557, 1886, 3376, 4946, 6502, 7946, 9184,
                                  10134, 10731, 10935, 10731, 10134, 9184, 7946, 6502, 4946, 3376, 1886, 557, -545, -1375, -1914, -2164, -2150,
                                  -1915, -1516, -1016, -477, 39, 483, 819, 1026, 1097, 1041, 880, 643, 364, 79, -181, -391, -533, -599, -590,
                                  -517, -393, -240, -78, 73, 198, 284, 0};
const uint16_t NXDN_0_2_FILTER_LEN = 82U;

static q15_t NXDN_ISINC_FILTER[] = {790, -1085, -1073, -553, 747, 2341, 3156, 2152, -893, -4915, -7834, -7536, -3102, 4441, 12354, 17394, 17394,
                                   12354, 4441, -3102, -7536, -7834, -4915, -893, 2152, 3156, 2341, 747, -553, -1073, -1085, 790};
const uint16_t NXDN_ISINC_FILTER_LEN = 32U;
#endif
#endif

#if defined(MODE_DSTAR)
// Generated using gaussfir(0.5, 4, 5) in MATLAB
static q15_t   GAUSSIAN_0_5_FILTER[] = {8, 104, 760, 3158, 7421, 9866, 7421, 3158, 760, 104, 8, 0};
const uint16_t GAUSSIAN_0_5_FILTER_LEN = 12U;
#endif

#if defined(MODE_P25)
// One symbol boxcar filter
static q15_t   BOXCAR5_FILTER[] = {12000, 12000, 12000, 12000, 12000, 0};
const uint16_t BOXCAR5_FILTER_LEN = 6U;
#endif

const size_t RX_CHANNEL = 0;
const size_t TX_CHANNEL = 0;

const size_t LATENCY_BLOCKS = 3;

const int32_t FM_DEVIATION = 550000;

// To allow for fine tuning of the deviation levels
const q15_t LEVEL_50PC_INVERTED = -128 * 128;
const q15_t LEVEL_40PC_INVERTED = -102 * 128;
const q15_t LEVEL_30PC_INVERTED = -77  * 128;
const q15_t LEVEL_100PC         =  255 * 128;

const unsigned int SAMPLES_TO_NETWORK = 720U;
const unsigned int MULTIMODEM_PACKET_SIZE = SAMPLES_TO_NETWORK * 3U + 8U;


CIO::CIO() :
m_sdrDevice(nullptr),
m_trace(false),
m_started(false),
#if defined(USE_DCBLOCKER)
m_dcFilter(),
m_dcState(),
#endif
#if defined(MODE_DSTAR)
m_gaussianFilter(),
m_gaussianState(),
#endif
#if defined(MODE_DMR)
m_rrc02Filter1(),
m_rrc02State1(),
#endif
#if defined(MODE_YSF)
m_rrc02Filter2(),
m_rrc02State2(),
#endif
#if defined(MODE_P25)
m_boxcar5Filter(),
m_boxcar5State(),
#endif
#if defined(MODE_NXDN)
#if defined(USE_NXDN_BOXCAR)
m_boxcar10Filter(),
m_boxcar10State(),
#else
m_nxdnFilter(),
m_nxdnISincFilter(),
m_nxdnState(),
m_nxdnISincState(),
#endif
#endif
m_power(0.0F),
m_txFreq(0U),
m_rxFreq(0U),
m_pocsagFreq(0U),
m_rxGain(50.0F),
m_txGain(30.0F)
{
#if defined(USE_DCBLOCKER)
  ::memset(m_dcState, 0x00U, 4U * sizeof(q31_t));
  m_dcFilter.numStages = DC_FILTER_STAGES;
  m_dcFilter.pState    = m_dcState;
  m_dcFilter.pCoeffs   = DC_FILTER;
  m_dcFilter.postShift = 0;
#endif

#if defined(MODE_DSTAR)
  ::memset(m_gaussianState, 0x00U, 40U * sizeof(q15_t));
  m_gaussianFilter.numTaps = GAUSSIAN_0_5_FILTER_LEN;
  m_gaussianFilter.pState  = m_gaussianState;
  m_gaussianFilter.pCoeffs = GAUSSIAN_0_5_FILTER;
#endif

#if defined(MODE_DMR)
  ::memset(m_rrc02State1, 0x00U, 70U * sizeof(q15_t));
  m_rrc02Filter1.numTaps = RRC_0_2_FILTER_LEN;
  m_rrc02Filter1.pState  = m_rrc02State1;
  m_rrc02Filter1.pCoeffs = RRC_0_2_FILTER;
#endif

#if defined(MODE_YSF)
  ::memset(m_rrc02State2, 0x00U, 70U * sizeof(q15_t));
  m_rrc02Filter2.numTaps = RRC_0_2_FILTER_LEN;
  m_rrc02Filter2.pState  = m_rrc02State2;
  m_rrc02Filter2.pCoeffs = RRC_0_2_FILTER;
#endif

#if defined(MODE_P25)
  ::memset(m_boxcar5State, 0x00U, 30U * sizeof(q15_t));
  m_boxcar5Filter.numTaps = BOXCAR5_FILTER_LEN;
  m_boxcar5Filter.pState  = m_boxcar5State;
  m_boxcar5Filter.pCoeffs = BOXCAR5_FILTER;
#endif

#if defined(MODE_NXDN)
#if defined(USE_NXDN_BOXCAR)
  ::memset(m_boxcar10State, 0x00U, 40U * sizeof(q15_t));
  m_boxcar10Filter.numTaps = BOXCAR10_FILTER_LEN;
  m_boxcar10Filter.pState  = m_boxcar10State;
  m_boxcar10Filter.pCoeffs = BOXCAR10_FILTER;
#else
  ::memset(m_nxdnState,      0x00U, 110U * sizeof(q15_t));
  ::memset(m_nxdnISincState, 0x00U, 60U * sizeof(q15_t));

  m_nxdnFilter.numTaps = NXDN_0_2_FILTER_LEN;
  m_nxdnFilter.pState  = m_nxdnState;
  m_nxdnFilter.pCoeffs = NXDN_0_2_FILTER;
  
  m_nxdnISincFilter.numTaps = NXDN_ISINC_FILTER_LEN;
  m_nxdnISincFilter.pState  = m_nxdnISincState;
  m_nxdnISincFilter.pCoeffs = NXDN_ISINC_FILTER;
#endif
#endif
}

CIO::~CIO()
{
  delete m_sdrDevice;
}

bool CIO::start(bool trace)
{
  if (m_started)
    return true;

  m_trace = trace;

  if (!m_sdrDevice)
    return false;

  m_started = m_sdrDevice->start(trace);;

  setMode(MMDVM_STATE::IDLE);

  return m_started;
}

void CIO::stop()
{
  if (m_sdrDevice)
    m_sdrDevice->stop();

  m_started = false;
}

void CIO::process(bool networkData)
{
  if (!m_started)
    return;

  if (!m_sdrDevice)
    return;

  m_sdrDevice->process();

  q15_t    samples[RX_BLOCK_SIZE];
  uint8_t  control[RX_BLOCK_SIZE];
  uint16_t rssi[RX_BLOCK_SIZE];

  while (m_sdrDevice->read(m_modemState, samples, rssi, control) == RX_BLOCK_SIZE) { 
    for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
      q31_t res2 = samples[i] * LEVEL_50PC_INVERTED;
      samples[i] = q15_t(__SSAT((res2 >> 15), 16));
    }

#if defined(USE_DCBLOCKER)
    q31_t q31Samples[RX_BLOCK_SIZE];
    ::arm_q15_to_q31(samples, q31Samples, RX_BLOCK_SIZE);

    q31_t dcValues[RX_BLOCK_SIZE];
    ::arm_biquad_cascade_df1_q31(&m_dcFilter, q31Samples, dcValues, RX_BLOCK_SIZE);

    q31_t dcLevel = 0;
    for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      dcLevel += dcValues[i];
    dcLevel /= RX_BLOCK_SIZE;

    q15_t offset = q15_t(__SSAT((dcLevel >> 16), 16));;

    q15_t dcSamples[RX_BLOCK_SIZE];
    for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      dcSamples[i] = samples[i] - offset;
#endif

    if (m_modemState == MMDVM_STATE::IDLE) {
#if defined(MODE_DSTAR)
        if (m_dstarEnable) {
            q15_t GMSKVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
            ::arm_fir_fast_q15(&m_gaussianFilter, dcSamples, GMSKVals, RX_BLOCK_SIZE);
#else
            ::arm_fir_fast_q15(&m_gaussianFilter, samples, GMSKVals, RX_BLOCK_SIZE);
#endif
            dstarRX.samples(GMSKVals, rssi, RX_BLOCK_SIZE);
        }
#endif

#if defined(MODE_P25)
        if (m_p25Enable) {
            q15_t P25Vals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
            ::arm_fir_fast_q15(&m_boxcar5Filter, dcSamples, P25Vals, RX_BLOCK_SIZE);
#else
            ::arm_fir_fast_q15(&m_boxcar5Filter, samples, P25Vals, RX_BLOCK_SIZE);
#endif
            p25RX.samples(P25Vals, rssi, RX_BLOCK_SIZE);
        }
#endif

#if defined(MODE_NXDN)
        if (m_nxdnEnable) {
            q15_t NXDNVals[RX_BLOCK_SIZE];
#if defined(USE_NXDN_BOXCAR)
#if defined(USE_DCBLOCKER)
            ::arm_fir_fast_q15(&m_boxcar10Filter, dcSamples, NXDNVals, RX_BLOCK_SIZE);
#else
            ::arm_fir_fast_q15(&m_boxcar10Filter, samples, NXDNVals, RX_BLOCK_SIZE);
#endif
#else
            q15_t NXDNValsTmp[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
            ::arm_fir_fast_q15(&m_nxdnFilter, dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
#else
            ::arm_fir_fast_q15(&m_nxdnFilter, samples, NXDNValsTmp, RX_BLOCK_SIZE);
#endif
            ::arm_fir_fast_q15(&m_nxdnISincFilter, NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);
#endif
            nxdnRX.samples(NXDNVals, rssi, RX_BLOCK_SIZE);
        }
#endif

#if defined(MODE_DMR)
        if (m_dmrEnable) {
            q15_t DMRVals[RX_BLOCK_SIZE];
            ::arm_fir_fast_q15(&m_rrc02Filter1, samples, DMRVals, RX_BLOCK_SIZE);

            if (m_duplex)
                dmrIdleRX.samples(DMRVals, RX_BLOCK_SIZE);
            else
                dmrDMORX.samples(DMRVals, rssi, RX_BLOCK_SIZE);
        }
#endif

#if defined(MODE_YSF)
        if (m_ysfEnable) {
            q15_t YSFVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
            ::arm_fir_fast_q15(&m_rrc02Filter2, dcSamples, YSFVals, RX_BLOCK_SIZE);
#else
            ::arm_fir_fast_q15(&m_rrc02Filter2, samples, YSFVals, RX_BLOCK_SIZE);
#endif
            ysfRX.samples(YSFVals, rssi, RX_BLOCK_SIZE);
        }
#endif

#if defined(MODE_FM)
      if (m_fmEnable) {
#if defined(USE_DCBLOCKER)
        fm.samples(dcSamples, rssi, RX_BLOCK_SIZE);
#else
        fm.samples(samples, rssi, RX_BLOCK_SIZE);
#endif
      }
#endif
    }

#if defined(MODE_DSTAR)
    else if (m_modemState == MMDVM_STATE::DSTAR) {
      if (m_dstarEnable) {
        q15_t GMSKVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_gaussianFilter, dcSamples, GMSKVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_gaussianFilter, samples, GMSKVals, RX_BLOCK_SIZE);
#endif
        dstarRX.samples(GMSKVals, rssi, RX_BLOCK_SIZE);
      }
    }
#endif

#if defined(MODE_DMR)
    else if (m_modemState == MMDVM_STATE::DMR) {
      if (m_dmrEnable) {
        q15_t DMRVals[RX_BLOCK_SIZE];
        ::arm_fir_fast_q15(&m_rrc02Filter1, samples, DMRVals, RX_BLOCK_SIZE);

        if (m_duplex) {
          // If the transmitter isn't on, use the DMR idle RX to detect the wakeup CSBKs
          if (m_tx)
            dmrRX.samples(DMRVals, rssi, control, RX_BLOCK_SIZE);
          else
            dmrIdleRX.samples(DMRVals, RX_BLOCK_SIZE);
        } else {
          dmrDMORX.samples(DMRVals, rssi, RX_BLOCK_SIZE);
        }
      }
    }
#endif

#if defined(MODE_YSF)
    else if (m_modemState == MMDVM_STATE::YSF) {
      if (m_ysfEnable) {
        q15_t YSFVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_rrc02Filter2, dcSamples, YSFVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_rrc02Filter2, samples, YSFVals, RX_BLOCK_SIZE);
#endif
        ysfRX.samples(YSFVals, rssi, RX_BLOCK_SIZE);
      }
    }
#endif

#if defined(MODE_P25)
    else if (m_modemState == MMDVM_STATE::P25) {
      if (m_p25Enable) {
        q15_t P25Vals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_boxcar5Filter, dcSamples, P25Vals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_boxcar5Filter, samples, P25Vals, RX_BLOCK_SIZE);
#endif
        p25RX.samples(P25Vals, rssi, RX_BLOCK_SIZE);
      }
    }
#endif

#if defined(MODE_NXDN)
    else if (m_modemState == MMDVM_STATE::NXDN) {
      if (m_nxdnEnable) {
        q15_t NXDNVals[RX_BLOCK_SIZE];
#if defined(USE_NXDN_BOXCAR)
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_boxcar10Filter, dcSamples, NXDNVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_boxcar10Filter, samples, NXDNVals, RX_BLOCK_SIZE);
#endif
#else
        q15_t NXDNValsTmp[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_nxdnFilter, dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_nxdnFilter, samples, NXDNValsTmp, RX_BLOCK_SIZE);
#endif
        ::arm_fir_fast_q15(&m_nxdnISincFilter, NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);
#endif
        nxdnRX.samples(NXDNVals, rssi, RX_BLOCK_SIZE);
      }
    }
#endif

#if defined(MODE_FM)
    else if (m_modemState == MMDVM_STATE::FM) {
#if defined(USE_DCBLOCKER)
      fm.samples(dcSamples, rssi, RX_BLOCK_SIZE);
#else
      fm.samples(samples, rssi, RX_BLOCK_SIZE);
#endif
    }
#endif
  }
}

void CIO::write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control)
{
  assert(samples != nullptr);
  assert(length > 0U);

  if (!m_started)
    return;

  if (m_sdrDevice)
    m_sdrDevice->write(mode, samples, length, control);
}

uint16_t CIO::getSpace() const
{
  return m_sdrDevice ? m_sdrDevice->getSpace() : 0;
}

void CIO::setMode(MMDVM_STATE state)
{
  m_modemState = state;
}

uint8_t CIO::setParameters()
{
  return m_sdrDevice ? m_sdrDevice->setParameters() : 0;
}

void CIO::setSoapyDeviceInfo(const std::string& type, const std::string& uri, unsigned int rxGain, unsigned int txGain)
{
  CIOSoapy* soapy = new CIOSoapy();
  assert(soapy);
  soapy->setDeviceInfo(type, uri, rxGain, txGain);

  delete m_sdrDevice;
  m_sdrDevice = soapy;
}

void CIO::setMultiModemAddress(std::string myAddress, unsigned short myPort, std::string modemAddress, unsigned short modemPort) {
  CSDRMulti* multi = new CSDRMulti();
  multi->setAddress(myAddress, myPort, modemAddress, modemPort);

  delete m_sdrDevice;
  m_sdrDevice = multi;
}

uint8_t CIO::setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
  return m_sdrDevice ? m_sdrDevice->setFrequency(power, txFreq, rxFreq, pocsagFreq) : 0;
}
