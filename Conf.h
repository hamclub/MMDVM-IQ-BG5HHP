/*
 *   Copyright (C) 2015-2023,2025,2026 by Jonathan Naylor G4KLX
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

#if !defined(CONF_H)
#define	CONF_H

#include <string>

class CConf
{
public:
	CConf(const std::string& file);
	~CConf();

	bool read();

	// The General section
	bool         getDaemon() const;

	// The Log section
	unsigned int getLogDisplayLevel() const;
	unsigned int getLogFileLevel() const;
	std::string  getLogFilePath() const;
	std::string  getLogFileRoot() const;
	bool         getLogFileRotate() const;

	// The Modem section
	std::string  getProtocol() const;
	std::string  getUARTPort() const;
	unsigned int getUARTSpeed() const;
	std::string  getModemAddress() const;
	uint16_t     getModemPort() const;
	std::string  getLocalAddress() const;
	uint16_t     getLocalPort() const;
	bool         getModemTrace() const;

	// The MMDVMHost section
	std::string  getNetworkHostAddress() const;
	unsigned short getNetworkHostPort() const;
	std::string  getNetworkLocalAddress() const;
	unsigned short getNetworkLocalPort() const;
	bool         getNetworkTrace() const;

private:
	std::string m_file;

	bool         m_daemon;

	unsigned int m_logDisplayLevel;
	unsigned int m_logFileLevel;
	std::string  m_logFilePath;
	std::string  m_logFileRoot;
	bool         m_logFileRotate;

	std::string  m_protocol;
	std::string  m_uartPort;
	unsigned int m_uartSpeed;
	std::string  m_modemAddress;
	uint16_t     m_modemPort; 
	std::string  m_localAddress;
	uint16_t     m_localPort;
	bool         m_modemTrace;

	std::string  m_networkHostAddress;
	unsigned short m_networkHostPort;
	std::string  m_networkLocalAddress;
	unsigned short m_networkLocalPort;
	bool         m_networkTrace;
};

#endif
