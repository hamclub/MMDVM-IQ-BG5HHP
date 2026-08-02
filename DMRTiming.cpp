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

#include "DMRTiming.h"

CDMRTiming::CDMRTiming(unsigned int rf_delay, int sample_delay) :
m_timingMutex(),
m_sampleDelay(0LL),
m_RFDelay(0LL),
m_timingInitialized(),
m_sampleCounter(),
m_lastSlot(),
m_timeBase(),
m_timeSlots()
{
    m_RFDelay = (long long)rf_delay * 1000000LL;

    // RF frontend delay for the Lime is larger than for the USRP by approx 104 microseconds
    m_sampleDelay = (long long)sample_delay * TIME_PER_SAMPLE; // used to adjust SYNC sample position in MMDVM 

    for (unsigned int i = 0U; i < MAX_MMDVM_CHANNELS; i++) {
        m_sampleCounter[i] = 0LL;
        m_lastSlot[i]      = 0LL;
        m_timeBase[i]      = 0LL;
        m_timingInitialized[i] = false;
    }
}

CDMRTiming::~CDMRTiming()
{
    for (unsigned int k = 0U; k < MAX_MMDVM_CHANNELS; k++) {
        m_timeSlots[k].clear();
    }
}

void CDMRTiming::lock()
{
    m_timingMutex.lock();
}

void CDMRTiming::unlock()
{
    m_timingMutex.unlock();
}

long long CDMRTiming::getTimeDelta(unsigned int cn)
{
    assert(cn < MAX_MMDVM_CHANNELS);

    return m_timeBase[cn] + m_sampleCounter[cn] * TIME_PER_SAMPLE;
}

void CDMRTiming::setTimer(long long value, unsigned int cn)
{
    assert(cn < MAX_MMDVM_CHANNELS);

    m_sampleCounter[cn] = 0LL;
    m_timeBase[cn] = value;

    if (m_timeBase[cn] > 4LL * SLOT_TIME)
        m_timingInitialized[cn] = true;
}

bool CDMRTiming::getInit(unsigned int cn)
{
    assert(cn < MAX_MMDVM_CHANNELS);

    return m_timingInitialized[cn];
}

uint8_t CDMRTiming::checkTime(unsigned int cn, bool time_base_received)
{
    assert(cn < MAX_MMDVM_CHANNELS);

    if (!time_base_received) // not the first sample in the batch
        m_sampleCounter[cn]++;

    if (m_timeSlots[cn].size() < 1U)
        return 0;

    long long sample_time = m_timeBase[cn] + m_sampleCounter[cn] * TIME_PER_SAMPLE - TOTAL_FILTER_DELAY - m_sampleDelay;
    DMRTimeSlot& s = m_timeSlots[cn].at(0);

    if (sample_time >= s.slotTime && s.slotSampleCounter == 0U) {
        s.slotSampleCounter++;
        return s.slotNo;
    } else if (sample_time >= s.slotTime) {
        if (s.slotSampleCounter >= (SAMPLES_PER_SLOT - 1U)) {
            m_timeSlots[cn].erase(m_timeSlots[cn].begin());
            return 0U;
        }

        s.slotSampleCounter++;
    }

    return 0U;
}

long long CDMRTiming::allocateSlot(uint8_t slot_no, int64_t& next_slot_timing_correction, unsigned int cn)
{
    assert(cn < MAX_MMDVM_CHANNELS);

    long long elapsed = getTimeDelta(cn);

    if (elapsed <= m_lastSlot[cn]) {
        if (cn == 0U)
            next_slot_timing_correction = m_lastSlot[cn] - elapsed;

        m_lastSlot[cn] = m_lastSlot[cn] + SLOT_TIME;
    } else if (m_lastSlot[cn] == 0LL) {
        m_lastSlot[cn] = elapsed;
    } else if ((elapsed - m_lastSlot[cn]) >= (1LL * SLOT_TIME)) {
        m_lastSlot[cn] = elapsed;
    } else {
        m_lastSlot[cn] = m_lastSlot[cn] + SLOT_TIME;
    }

    long long nsec = m_lastSlot[cn] + m_RFDelay;
    DMRTimeSlot s;
    s.slotNo = slot_no;
    s.slotTime = nsec;
    s.slotSampleCounter = 0;
    m_timeSlots[cn].push_back(s);

    return nsec;
}
