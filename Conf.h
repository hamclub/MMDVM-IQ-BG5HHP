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
#include <cstdint>

class CConf
{
public:
	CConf(const std::string& file);
	~CConf();

	bool read();

	// The General section
	bool         getDaemon() const;

	// The Log section
	unsigned int getLogFileLevel() const;
	std::string  getLogFilePath() const;
	std::string  getLogFileRoot() const;
	unsigned int getLogDisplayLevel() const;
	unsigned int getLogMQTTLevel() const;

    // The MQTT section
    std::string    getMQTTHost() const;
    unsigned short getMQTTPort() const;
    unsigned int   getMQTTKeepalive() const;
    std::string    getMQTTName() const;
    bool           getMQTTAuthEnabled() const;
    std::string    getMQTTUsername() const;
    std::string    getMQTTPassword() const;

	// The Modem section
	std::string  getModemType() const;
	std::string  getModemURI() const;
	bool         getModemTrace() const;

	unsigned char getModemVersion() const;

	// The MMDVMHost section
	std::string  getNetworkHostAddress() const;
	unsigned short getNetworkHostPort() const;
	std::string  getNetworkLocalAddress() const;
	unsigned short getNetworkLocalPort() const;
	bool         getNetworkTrace() const;

private:
	std::string m_file;

	bool         m_daemon;

	unsigned int m_logFileLevel;
	std::string  m_logFilePath;
	std::string  m_logFileRoot;

	unsigned int m_logDisplayLevel;
	unsigned int m_logMQTTLevel;

    std::string  m_mqttHost;
    unsigned short m_mqttPort;
    unsigned int m_mqttKeepalive;
    std::string  m_mqttName;
    bool         m_mqttAuthEnabled;
    std::string  m_mqttUsername;
    std::string  m_mqttPassword;

    std::string  m_modemType;
    std::string  m_modemURI;
	bool         m_modemTrace;
    unsigned char m_modemVersion;

	std::string  m_networkHostAddress;
	unsigned short m_networkHostPort;
	std::string  m_networkLocalAddress;
	unsigned short m_networkLocalPort;
	bool         m_networkTrace;
};

#endif
