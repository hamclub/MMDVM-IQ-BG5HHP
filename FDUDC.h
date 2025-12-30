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

#if !defined(FDUDC_H)
#define FDUDC_H

#include "IQSample.h"

#include <arm_math.h>

class IFDUDC {
public:
    virtual ~IFDUDC() = 0;

    virtual void setCallback(void (*callback)(const IQSample<float32_t>& sample)) = 0;

    virtual void process(const IQSample<float32_t>& sample) = 0;

private:
};

class CFDUDCDummy : public IFDUDC {
public:
    CFDUDCDummy();
    virtual ~CFDUDCDummy();

    virtual void setCallback(void (*callback)(const IQSample<float32_t>& sample));

    virtual void process(const IQSample<float32_t>& sample);

private:
    void (*m_callback)(const IQSample<float32_t>& sample);
};

#endif
