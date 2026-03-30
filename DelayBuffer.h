/*
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
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

#if !defined(DELAYBUFFER_H)
#define DELAYBUFFER_H

template <typename TDATATYPE>
class CDelayBuffer {
public:
    CDelayBuffer(size_t length, TDATATYPE initial_value):
    m_buffer(new TDATATYPE[length]),
    m_length(length),
    m_index(0)
    {
        for (size_t i = 0; i < length; i++) {
            m_buffer[i] = initial_value;
        }
    }

    TDATATYPE process(TDATATYPE input)
    {
        if (m_length == 0) {
            return input;
        }
        auto delayed = m_buffer[m_index];
        m_buffer[m_index] = input;
        if (++m_index >= m_length) {
            m_index = 0;
        }
        return delayed;
    }

private:
    TDATATYPE *m_buffer;
    size_t m_length;
    size_t m_index;
};

#endif
