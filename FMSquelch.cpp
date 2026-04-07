/*
 *   Copyright (C) 2020,2026 by Jonathan Naylor G4KLX
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

#include "Config.h"

#if defined(MODE_FM)

#include "Globals.h"
#include "FMSquelch.h"

const uint8_t MAX_COUNT = 4U;


CFMSquelch::CFMSquelch() :
m_highThreshold(0U),
m_lowThreshold(0U),
m_count(0U),
m_state(false)
{
}

CFMSquelch::~CFMSquelch()
{
}

void CFMSquelch::setParams(uint8_t highThreshold, uint8_t lowThreshold)
{
	m_highThreshold = highThreshold;
	m_lowThreshold  = lowThreshold;
}

bool CFMSquelch::process(uint16_t rssi)
{
	if (m_state) {
		if (rssi <= m_lowThreshold) {
			m_count--;

			if (m_count == 0U)
				m_state = false;
		} else {
			m_state = MAX_COUNT;
		}
	} else {
		if (rssi >= m_highThreshold) {
			m_count++;

			if (m_count >= MAX_COUNT)
				m_state = true;
		}
	}

	return m_state;
}

#endif
