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

#if !defined(SERIALPORT_H)
#define  SERIALPORT_H

#include "Config.h"
#include "Globals.h"
#include "RingBuffer.h"
#include "Socket.h"


class CSerialPort {
public:
  CSerialPort();
  ~CSerialPort();

  bool start(const std::string& myAddress, unsigned short myPort, const std::string& hostAddress, unsigned short hostPort, bool debug);

  void process();

  void stop();

#if defined(MODE_DSTAR)
  void writeDStarHeader(const uint8_t* header, uint8_t length);
  void writeDStarData(const uint8_t* data, uint8_t length);
  void writeDStarLost();
  void writeDStarEOT();
#endif

#if defined(MODE_DMR)
  void writeDMRData(bool slot, const uint8_t* data, uint8_t length);
  void writeDMRLost(bool slot);
#endif

#if defined(MODE_YSF)
  void writeYSFData(const uint8_t* data, uint8_t length);
  void writeYSFLost();
#endif

#if defined(MODE_P25)
  void writeP25Hdr(const uint8_t* data, uint8_t length);
  void writeP25Ldu(const uint8_t* data, uint8_t length);
  void writeP25Lost();
#endif

#if defined(MODE_NXDN)
  void writeNXDNData(const uint8_t* data, uint8_t length);
  void writeNXDNLost();
#endif

#if defined(MODE_FM)
  void writeFMData(const uint8_t* data, uint16_t length);
  void writeFMStatus(uint8_t status);
  void writeFMRSSI(uint16_t rssi);
  void writeFMEOT();
#endif

  void setVersion(unsigned char version) { m_version = version; };

private:
  uint8_t   m_buffer[512U];
  uint16_t  m_ptr;
  uint16_t  m_len;
  CSocket   m_socket;
  unsigned char m_version;
  bool      m_trace;

  void    sendACK(uint8_t type);
  void    sendNAK(uint8_t type, uint8_t err);
  void    getStatus1();
  void    getStatus();
  void    getVersion1();
  void    getVersion();
  uint8_t setFrequency(const uint8_t* data, uint16_t length);
  uint8_t setConfig1(const uint8_t* data, uint16_t length);
  uint8_t setConfig(const uint8_t* data, uint16_t length);
  uint8_t setMode(const uint8_t* data, uint16_t length);
  void    setMode(MMDVM_STATE modemState);
  void    processMessage(uint8_t type, const uint8_t* data, uint16_t length);

#if defined(MODE_FM)
  uint8_t setFMParams1(const uint8_t* data, uint16_t length);
  uint8_t setFMParams2(const uint8_t* data, uint16_t length);
  uint8_t setFMParams3(const uint8_t* data, uint16_t length);
  uint8_t setFMParams4(const uint8_t* data, uint16_t length);
#endif

  void dump(const char* text, const uint8_t* data, uint16_t length) const;
};

#endif
