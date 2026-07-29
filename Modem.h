/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 026 by Shawn Chain BG5HHP
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

#if !defined(MODEM_H)
#define  MODEM_H

#include "Config.h"

#include "Globals.h"

class CModem {
public:
    //
    // Modem shared states
    ///////////////////////////////////////////////////////////////////////////
    unsigned int m_channel = 0;

    MMDVM_STATE m_modemState = MMDVM_STATE::IDLE;

    bool m_dstarEnable  = true;
    bool m_dmrEnable    = true;
    bool m_ysfEnable    = true;
    bool m_p25Enable    = true;
    bool m_nxdnEnable   = true;
    bool m_pocsagEnable = true;
    bool m_fmEnable     = true;

    bool m_duplex = true;

    bool m_tx  = false;
    bool m_dcd = false;

#if defined(MODE_DSTAR)
    CDStarRX dstarRX;
    CDStarTX dstarTX;
#endif

#if defined(MODE_DMR)
    CDMRIdleRX dmrIdleRX;
    CDMRRX dmrRX;
    CDMRTX dmrTX;

    CDMRDMORX dmrDMORX;
    CDMRDMOTX dmrDMOTX;
#endif

#if defined(MODE_YSF)
    CYSFRX ysfRX;
    CYSFTX ysfTX;
#endif

#if defined(MODE_P25)
    CP25RX p25RX;
    CP25TX p25TX;
#endif

#if defined(MODE_NXDN)
    CNXDNRX nxdnRX;
    CNXDNTX nxdnTX;
#endif

#if defined(MODE_POCSAG)
    CPOCSAGTX  pocsagTX;
#endif

#if defined(MODE_FM)
    CFM    fm;
#endif

    CCWIdTX cwIdTX;

    ///////////////////////////////////////////////////////////////////////////

    // CIO* m_io = nullptr;

    CModem() {
    }

    ~CModem() {
    }
    
    unsigned int getChannel() {
        return m_channel;
    }

    void setState(MMDVM_STATE state) {
        m_modemState = state;
    }

    bool checkState(MMDVM_STATE state) {
        return m_modemState == state;
    }

    void setModeEnable(MMDVM_STATE mode, bool enabled) {
        switch (mode) {
            case MMDVM_STATE::DSTAR:
                m_dstarEnable = enabled;
                break;

            case MMDVM_STATE::DMR:
                m_dmrEnable = enabled;
                break;

            case MMDVM_STATE::YSF:
                m_ysfEnable = enabled;
                break;

            case MMDVM_STATE::P25:
                m_p25Enable  = enabled;
                break;

            case MMDVM_STATE::NXDN:
                m_nxdnEnable = enabled;
                break;

            case MMDVM_STATE::POCSAG:
                m_pocsagEnable = enabled;
                break;

            case MMDVM_STATE::FM:
                m_fmEnable = enabled;
                break;

            default:
                break;
        }
    }

    bool isModeEnabled(MMDVM_STATE mode) {
        switch (mode) {
            case MMDVM_STATE::DSTAR:
                return m_dstarEnable;

            case MMDVM_STATE::DMR:
                return m_dmrEnable;

            case MMDVM_STATE::YSF:
                return m_ysfEnable;

            case MMDVM_STATE::P25:
                return m_p25Enable;

            case MMDVM_STATE::NXDN:
                return m_nxdnEnable;

            case MMDVM_STATE::POCSAG:
                return m_pocsagEnable;

            case MMDVM_STATE::FM:
                return m_fmEnable;

            default:
                return false;
        }
    }
};

#endif