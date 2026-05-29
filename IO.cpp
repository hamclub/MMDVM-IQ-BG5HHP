/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
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

#include <cstdio>
#include <cassert>

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

CIO::CIO() :
m_trace(false),
m_started(false),
m_rxBuffer(RX_RINGBUFFER_SIZE, "IO RX Buffer"),
m_txBuffer(TX_RINGBUFFER_SIZE, "IO TX Buffer"),
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
m_soapyTXFreq(0.0),
m_soapyRXFreq(0.0),
m_soapyPocsagFreq(0.0),
m_soapyInit(false),
m_timestamped(false),
m_latencyNs(0LL),
m_phase(0U),
m_prevRXIQSample(0.0F, 0.0F),
m_delayedTXBuffer(nullptr),
m_buffer(),
m_fdudc(nullptr),
m_soapyDeviceType("sx"),
m_soapyDeviceURI(),
m_device(nullptr),
m_rxStream(nullptr),
m_txStream(nullptr)
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
}

bool CIO::start(bool trace)
{
  if (m_started)
    return true;

  m_trace = trace;

  m_started = true;

  setMode(MMDVM_STATE::IDLE);

  return true;
}

void CIO::stop()
{
  delete m_fdudc;
  m_fdudc = nullptr;

  delete m_delayedTXBuffer;
  m_delayedTXBuffer = nullptr;

  if (m_device != nullptr) {
    assert(m_rxStream != nullptr);
    assert(m_txStream != nullptr);

    if (m_soapyInit) {
      m_device->deactivateStream(m_rxStream, 0, 0);
      m_device->deactivateStream(m_txStream, 0, 0);
    }

    m_device->closeStream(m_rxStream);
    m_device->closeStream(m_txStream);

    SoapySDR::Device::unmake(m_device);
  }

  m_rxStream = nullptr;
  m_txStream = nullptr;
  m_device   = nullptr;

  m_soapyInit = false;
}

void CIO::process()
{
  if (!m_started)
    return;

  if (m_device == nullptr)
    return;

  assert(m_device != nullptr);
  assert(m_rxStream != nullptr);
  assert(m_txStream != nullptr);

  if (!m_soapyInit) {
    m_device->activateStream(m_rxStream);
    m_device->activateStream(m_txStream);

    if (!m_timestamped) {
      // Write initial zeros to transmit buffer to start streams
      for (size_t i = 0; i < m_buffer.size(); i++)
        m_buffer[i] = { 0.0F, 0.0F };

      for (size_t i = 0; i < LATENCY_BLOCKS; i++) {
        void* buffs[1] = { (void*)m_buffer.data() };
        int flags = 0;
        int ret = m_device->writeStream(m_txStream, buffs, m_buffer.size(), flags);
        if (ret <= 0) {
          LogError("TX stream start error: %d (%s)", ret, SoapySDR_errToStr(ret));
          break;
        }
      }
    }

    m_soapyInit = true;
  }

  void *buffs[1] = {(void*)m_buffer.data()};
  long long timeNs = 0LL;

  if (m_soapyInit) {
    int flags = 0;
    int ret = m_device->readStream(m_rxStream, buffs, m_buffer.size(), flags, timeNs);
    if (ret > 0) {
      processIQBlock();
    } else {
      LogError("RX stream error: %d (%s)", ret, SoapySDR_errToStr(ret));
      m_soapyInit = false;
    }
  }

  if (m_soapyInit) {
    int flags = 0;
    if (m_timestamped) {
      timeNs += m_latencyNs;
      flags   = SOAPY_SDR_HAS_TIME;
    }

    int ret = m_device->writeStream(m_txStream, buffs, m_buffer.size(), flags, timeNs);
    if (ret <= 0) {
      LogError("TX stream error: %d (%s)", ret, SoapySDR_errToStr(ret));
      m_soapyInit = false;
    }
  }

  if (!m_soapyInit) {
    m_device->deactivateStream(m_rxStream);
    m_device->deactivateStream(m_txStream);
    return;
  }

  // Switch off the transmitter if needed
  if (!m_txBuffer.hasData() && m_tx) {
    m_tx = false;
    LogMessage("TX OFF");
    m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "NONE");
  }

  while (m_rxBuffer.dataSize() >= RX_BLOCK_SIZE) {
    q15_t    samples[RX_BLOCK_SIZE];
    uint8_t  control[RX_BLOCK_SIZE];
    uint16_t rssi[RX_BLOCK_SIZE];

    for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
      RXSample sample;
      m_rxBuffer.getData(sample);
      control[i] = sample.m_control;
      rssi[i]    = sample.m_rssi;

      q31_t res2 = sample.m_sample * LEVEL_50PC_INVERTED;
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

void CIO::processIQBlock()
{
  assert(m_fdudc != nullptr);
  assert(m_delayedTXBuffer != nullptr);

  // Mute the receiver when transmitting in simplex mode
  if (m_tx && !m_duplex) {
    for (auto& d : m_buffer)
      d = {0.0F, 0.0F};
  }

  // Insert a channel filter here

  m_fdudc->process(m_buffer, [this](std::complex<float> rxIQSample) {
    std::complex<float> txIQSample = {0.0F, 0.0F};
    TXSample txSample = {0, MARK_NONE};

    if (m_txBuffer.getData(txSample)) {
      // Modulate TX
      m_phase += txSample.m_sample * FM_DEVIATION;
      float ph = m_phase * float(M_PI / 0x80000000UL);
      txIQSample = std::polar(m_power, ph);
    }

    // Demodulate RX
    float d = std::arg(rxIQSample * std::conj(m_prevRXIQSample));
    m_prevRXIQSample = rxIQSample;

    // Scale -pi...pi to -4096...4096
    d *= 4096.0F / M_PI;

    txSample = m_delayedTXBuffer->process(txSample);

    RXSample rxSample = {
      .m_sample  = q15_t(d + 0.5F),
      .m_rssi    = uint16_t(100000000.0F * std::norm(rxIQSample)),
      .m_control = txSample.m_control
    };

    m_rxBuffer.addData(rxSample);

    return txIQSample;
  });
}

void CIO::write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control)
{
  assert(samples != nullptr);
  assert(length > 0U);

  if (!m_started)
    return;

  if (!m_tx) {
      m_tx = true;
      LogMessage("TX ON");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "TX");
  }

  q15_t txLevel;
  switch (mode) {
    case MMDVM_STATE::FM:
      txLevel = LEVEL_100PC;
      break;
    default:
      txLevel = LEVEL_40PC_INVERTED;
      break;
  }

  for (uint16_t i = 0U; i < length; i++) {
    q31_t res1 = samples[i] * txLevel;
    q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));

    if (control == nullptr)
      m_txBuffer.addData({res2, MARK_NONE});
    else
      m_txBuffer.addData({res2, control[i]});
  }
}

uint16_t CIO::getSpace() const
{
  return m_txBuffer.freeSpace();
}

void CIO::setMode(MMDVM_STATE state)
{
  // Do we need to stop or pause the transmit stream to do this?
  if (m_device != nullptr) {
    if ((state == MMDVM_STATE::POCSAG) && (m_modemState != MMDVM_STATE::POCSAG))
      m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyPocsagFreq);

    if ((m_modemState == MMDVM_STATE::POCSAG) && (state != MMDVM_STATE::POCSAG))
      m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXFreq);
  }

  m_modemState = state;
}

uint8_t CIO::setParameters()
{
  stop();

  SoapySDR::Kwargs devArgs;
  SoapySDR::Kwargs rxArgs;
  SoapySDR::Kwargs txArgs;

  unsigned int resampNum = 4U;
  unsigned int resampDen = 25U;
  size_t blockSize = 512U;
  size_t iqHWDelay = 10;

  const unsigned int resampLen = 11U;
  const unsigned int rxIfNum = 1U, rxIfDen = 12U;
  const unsigned int txIfNum = 1U, txIfDen = 12U;

  const char* PLUTO_DEFAULT_URI = "ip:pluto.local";
  const char* LIME_DEFAULT_URI  = "index=0";         // eg: addr=1111:2222 or serial=xxxxxxxx

  if (m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? PLUTO_DEFAULT_URI : m_soapyDeviceURI.c_str();

    devArgs["driver"] = "plutosdr";
    rxArgs["uri"]     = uri;

    m_timestamped = false;

    LogMessage("Using Pluto SDR driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("limesdr") == 0 || m_soapyDeviceType.compare("lime") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? LIME_DEFAULT_URI : m_soapyDeviceURI.c_str();

    resampNum = 2U;
    resampDen = 50U;
    blockSize = 2048U;
    iqHWDelay = 50U;

    devArgs["driver"] = "lime";
    rxArgs["uri"]     = uri;
    rxArgs["latency"] = "0";
    txArgs["latency"] = "0";

    m_timestamped = true;

    LogMessage("Using Lime SDR driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("mucell") == 0) {
    resampNum = 4U;
    resampDen = 25U;
    blockSize = 512U;
    iqHWDelay = 10U;

    devArgs["driver"] = "mucell";

    m_timestamped = true;

    LogMessage("Using muCell driver");
  } else {
    resampNum = 4U;
    resampDen = 25U;
    blockSize = 512U;
    iqHWDelay = 10U;

    devArgs["driver"] = "sx";

    m_timestamped = true;

    LogMessage("Using SX1255 driver");
  }

  m_buffer.resize(blockSize);

  size_t latencySamples = (blockSize * LATENCY_BLOCKS + iqHWDelay) * resampNum / resampDen + resampLen;

  m_delayedTXBuffer = new CDelayBuffer<TXSample>(latencySamples, {0, 0U});

  const double samplerate = 24000.0 * double(resampDen) / double(resampNum);

  m_latencyNs = (long long)std::round(1e9 / samplerate * (double)(blockSize * LATENCY_BLOCKS));

  m_fdudc = new CFDUDC(resampNum, resampDen, rxIfNum, rxIfDen, txIfNum, txIfDen, resampLen, 0.5F);

  m_soapyTXFreq     = double(m_txFreq)     - samplerate * double(txIfNum) / double(txIfDen);
  m_soapyPocsagFreq = double(m_pocsagFreq) - samplerate * double(txIfNum) / double(txIfDen);
  m_soapyRXFreq     = double(m_rxFreq) - samplerate * double(rxIfNum) / double(rxIfDen);

  LogMessage("SDR Parameters");
  LogMessage("  Sample Rate:      %.0f samples/sec", samplerate);
  LogMessage("  Latency    :      %.2f ms", (double)m_latencyNs / 1e6);
  LogMessage("  TX Frequency:     %.0f Hz", m_soapyTXFreq);
  LogMessage("  RX Frequency:     %.0f Hz", m_soapyRXFreq);
  LogMessage("  POCSAG Frequency: %.0f Hz", m_soapyPocsagFreq);

  try {
    m_device = SoapySDR::Device::make(devArgs);
    assert(m_device != nullptr);

    m_device->setSampleRate(SOAPY_SDR_RX, RX_CHANNEL, samplerate);
    m_device->setSampleRate(SOAPY_SDR_TX, TX_CHANNEL, samplerate);

    m_device->setFrequency(SOAPY_SDR_RX, RX_CHANNEL, m_soapyRXFreq);
    m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXFreq);

    if (m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "A_BALANCED");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "A");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, 30.0);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, 89.75);
    } else if (m_soapyDeviceType.compare("limesdr") == 0 || m_soapyDeviceType.compare("lime") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "LNAH");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "BAND1");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, 30.0);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, 60.0);
    } else {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "RX");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "NONE");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, 50.0);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, 30.0);
    }

    m_rxStream = m_device->setupStream(SOAPY_SDR_RX, "CF32", {RX_CHANNEL}, rxArgs);
    m_txStream = m_device->setupStream(SOAPY_SDR_TX, "CF32", {TX_CHANNEL}, txArgs);

    assert(m_rxStream != nullptr);
    assert(m_txStream != nullptr);

    m_soapyInit = false;

    LogMessage("SoapySDR device setup done");
  } catch (std::runtime_error &e) {
    LogError("Error setting up SoapySDR device: %s", e.what());
    return 4U;
  }

  return 0U;
}

void CIO::setSoapyDeviceInfo(const std::string& type, const std::string& uri)
{
  m_soapyDeviceType = type;
  m_soapyDeviceURI  = uri;
}

uint8_t CIO::setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
  if ((txFreq < MIN_RF_FREQUENCY) || (txFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((rxFreq < MIN_RF_FREQUENCY) || (rxFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((pocsagFreq < MIN_RF_FREQUENCY) || (pocsagFreq > MAX_RF_FREQUENCY))
    return 4U;

  m_power      = float(power) / 255.0F;
  m_txFreq     = txFreq;
  m_rxFreq     = rxFreq;
  m_pocsagFreq = pocsagFreq;

  return 0U;
}
