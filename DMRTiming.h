/*
 *   Copyright (C) 2023-2026 by Adrian Musceac YO8RZZ
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

#ifndef DMRTIMING_H
#define DMRTIMING_H

#include "Constants.h"
#include "Mutex.h"

#include <cstdint>
#include <vector>
#include <cassert>

struct DMRTimeSlot {
    uint8_t   slotNo    = 0;
    long long slotTime  = 0;
    uint16_t  slotSampleCounter = 0;
};

class CDMRTiming
{
public:
    CDMRTiming(unsigned int rf_delay, int sample_delay);
    ~CDMRTiming();

    void      lock();
    void      unlock();

    void      setTimer(long long value, unsigned int cn = 0U);
    bool      getInit(unsigned int cn = 0U);
    uint8_t   checkTime(unsigned int cn = 0U, bool time_base_received = false);
    long long allocateSlot(uint8_t slot_no, int64_t& next_slot_timing_correction, unsigned int cn = 0U);

private:
    CMutex    m_timingMutex;

    long long m_sampleDelay;
    long long m_RFDelay;
    bool      m_timingInitialized[MAX_MMDVM_CHANNELS];
    long long m_sampleCounter[MAX_MMDVM_CHANNELS];
    long long m_lastSlot[MAX_MMDVM_CHANNELS];
    long long m_timeBase[MAX_MMDVM_CHANNELS];
    std::vector<DMRTimeSlot> m_timeSlots[MAX_MMDVM_CHANNELS];

    long long getTimeDelta(unsigned int cn = 0U);
};

#endif // DMRTIMING_H
