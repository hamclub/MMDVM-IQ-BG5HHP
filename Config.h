/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2025 by Jonathan Naylor G4KLX
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

#if !defined(CONFIG_H)
#define  CONFIG_H

// Allow for the selection of which modes to compile into the firmware. This is particularly useful for processors
// which have limited code space and processing power.

// Enable D-Star support.
#define MODE_DSTAR

// Enable DMR support.
#define MODE_DMR

// Enable System Fusion support.
#define MODE_YSF

// Enable P25 phase 1 support.
#define MODE_P25

// Enable NXDN support, the boxcar filter sometimes improves the performance of NXDN receive on some systems.
#define MODE_NXDN
#define USE_NXDN_BOXCAR

// Enable POCSAG support.
#define MODE_POCSAG

// Enable FM support.
#define MODE_FM

// To reduce CPU load, you can remove the DC blocker by commenting out the next line
#define USE_DCBLOCKER

#endif
