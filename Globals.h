/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
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

#if !defined(GLOBALS_H)
#define  GLOBALS_H

#include <cstdint>

#include <arm_math.h>

enum class MMDVM_STATE : uint8_t {
  IDLE   = 0U,
  DSTAR  = 1U,
  DMR    = 2U,
  YSF    = 3U,
  P25    = 4U,
  NXDN   = 5U,
  POCSAG = 6U,
  FM     = 10U,

  // Dummy states start at 90
  CWID   = 97U
};

#include "MQTTConnection.h"
#include "SerialPort.h"
#include "DMRIdleRX.h"
#include "DMRDMORX.h"
#include "DMRDMOTX.h"
#include "DStarRX.h"
#include "DStarTX.h"
#include "DMRRX.h"
#include "DMRTX.h"
#include "YSFRX.h"
#include "YSFTX.h"
#include "P25RX.h"
#include "P25TX.h"
#include "NXDNRX.h"
#include "NXDNTX.h"
#include "POCSAGTX.h"
#include "CWIdTX.h"
#include "Log.h"
#include "IO.h"
#include "FM.h"

const uint32_t MIN_RF_FREQUENCY = 420000000U;
const uint32_t MAX_RF_FREQUENCY = 450000000U;

const uint8_t  MARK_SLOT1 = 0x08U;
const uint8_t  MARK_SLOT2 = 0x04U;
const uint8_t  MARK_NONE  = 0x00U;

const uint16_t RX_BLOCK_SIZE = 2U;

const uint16_t RX_RINGBUFFER_SIZE = 600U;
const uint16_t TX_RINGBUFFER_SIZE = 240U;

const uint16_t TX_BUFFER_LEN = 4000U;

extern MMDVM_STATE m_modemState;

extern bool m_dstarEnable;
extern bool m_dmrEnable;
extern bool m_ysfEnable;
extern bool m_p25Enable;
extern bool m_nxdnEnable;
extern bool m_pocsagEnable;
extern bool m_fmEnable;

extern bool m_duplex;

extern bool m_tx;
extern bool m_dcd;

extern CSerialPort serial;
extern CIO io;

#if defined(MODE_DSTAR)
extern CDStarRX dstarRX;
extern CDStarTX dstarTX;
#endif

#if defined(MODE_DMR)
extern CDMRIdleRX dmrIdleRX;
extern CDMRRX dmrRX;
extern CDMRTX dmrTX;

extern CDMRDMORX dmrDMORX;
extern CDMRDMOTX dmrDMOTX;
#endif

#if defined(MODE_YSF)
extern CYSFRX ysfRX;
extern CYSFTX ysfTX;
#endif

#if defined(MODE_P25)
extern CP25RX p25RX;
extern CP25TX p25TX;
#endif

#if defined(MODE_NXDN)
extern CNXDNRX nxdnRX;
extern CNXDNTX nxdnTX;
#endif

#if defined(MODE_POCSAG)
extern CPOCSAGTX  pocsagTX;
#endif

#if defined(MODE_FM)
extern CFM    fm;
#endif

extern CCWIdTX cwIdTX;

extern CMQTTConnection* m_mqtt;

#endif
