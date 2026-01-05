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

#include "DStarDefines.h"
#include "Conf.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

const int BUFFER_SIZE = 500;

enum class SECTION {
	NONE,
	GENERAL,
	LOG,
	MODEM,
	MMDVM_HOST
};

CConf::CConf(const std::string& file) :
m_file(file),
m_daemon(false),
m_logDisplayLevel(0U),
m_logFileLevel(0U),
m_logFilePath(),
m_logFileRoot(),
m_logFileRotate(true),
m_modemPort(),
m_modemSpeed(921600U),
m_modemTrace(false),
m_networkHostAddress("127.0.0.1"),
m_networkHostPort(3335U),
m_networkLocalAddress("127.0.0.1"),
m_networkLocalPort(3334U),
m_networkTrace(false)
{
}

CConf::~CConf()
{
}

bool CConf::read()
{
	FILE* fp = ::fopen(m_file.c_str(), "rt");
	if (fp == nullptr) {
		::fprintf(stderr, "Couldn't open the .ini file - %s\n", m_file.c_str());
		return false;
	}

	SECTION section = SECTION::NONE;

	char buffer[BUFFER_SIZE];
	while (::fgets(buffer, BUFFER_SIZE, fp) != nullptr) {
		if (buffer[0U] == '#')
			continue;

		if (buffer[0U] == '[') {
			if (::strncmp(buffer, "[General]", 9U) == 0)
				section = SECTION::GENERAL;
			else if (::strncmp(buffer, "[Log]", 5U) == 0)
				section = SECTION::LOG;
			else if (::strncmp(buffer, "[Modem]", 7U) == 0)
				section = SECTION::MODEM;
			else if (::strncmp(buffer, "[MMDVM Host]", 12U) == 0)
				section = SECTION::MMDVM_HOST;
			else
				section = SECTION::NONE;

			continue;
		}

		char* key = ::strtok(buffer, " \t=\r\n");
		if (key == nullptr)
			continue;

		char* value = ::strtok(nullptr, "\r\n");
		if (value == nullptr)
			continue;

		// Remove quotes from the value
		size_t len = ::strlen(value);
		if (len > 1U && *value == '"' && value[len - 1U] == '"') {
			value[len - 1U] = '\0';
			value++;
		} else {
			char *p;

			// if value is not quoted, remove after # (to make comment)
			if ((p = strchr(value, '#')) != nullptr)
				*p = '\0';

			// remove trailing tab/space
			for (p = value + strlen(value) - 1U; p >= value && (*p == '\t' || *p == ' '); p--)
				*p = '\0';
		}

		if (section == SECTION::GENERAL) {
			if (::strcmp(key, "Daemon") == 0)
				m_daemon = ::atoi(value) == 1;
		} else if (section == SECTION::LOG) {
			if (::strcmp(key, "FilePath") == 0)
				m_logFilePath = value;
			else if (::strcmp(key, "FileRoot") == 0)
				m_logFileRoot = value;
			else if (::strcmp(key, "FileLevel") == 0)
				m_logFileLevel = (unsigned int)::atoi(value);
			else if (::strcmp(key, "DisplayLevel") == 0)
				m_logDisplayLevel = (unsigned int)::atoi(value);
			else if (::strcmp(key, "FileRotate") == 0)
				m_logFileRotate = ::atoi(value) == 1;
		} else if (section == SECTION::MODEM) {
			if (::strcmp(key, "Port") == 0)
				m_modemPort = value;
			else if (::strcmp(key, "Speed") == 0)
				m_modemSpeed = (unsigned int)::atoi(value);
			else if (::strcmp(key, "Trace") == 0)
				m_modemTrace = ::atoi(value) == 1;
		} else if (section == SECTION::MMDVM_HOST) {
			if (::strcmp(key, "HostAddress") == 0)
				m_networkHostAddress = value;
			else if (::strcmp(key, "HostPort") == 0)
				m_networkHostPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "LocalAddress") == 0)
				m_networkLocalAddress = value;
			else if (::strcmp(key, "LocalPort") == 0)
				m_networkLocalPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "Trace") == 0)
				m_networkTrace = ::atoi(value) == 1;
		}
	}

	::fclose(fp);

	return true;
}

bool CConf::getDaemon() const
{
	return m_daemon;
}

unsigned int CConf::getLogDisplayLevel() const
{
	return m_logDisplayLevel;
}

unsigned int CConf::getLogFileLevel() const
{
	return m_logFileLevel;
}

std::string CConf::getLogFilePath() const
{
	return m_logFilePath;
}

std::string CConf::getLogFileRoot() const
{
	return m_logFileRoot;
}

bool CConf::getLogFileRotate() const
{
	return m_logFileRotate;
}

std::string CConf::getModemPort() const
{
	return m_modemPort;
}

unsigned int CConf::getModemSpeed() const
{
	return m_modemSpeed;
}

bool CConf::getModemTrace() const
{
	return m_modemTrace;
}

std::string CConf::getNetworkHostAddress() const
{
	return m_networkHostAddress;
}

unsigned short CConf::getNetworkHostPort() const
{
	return m_networkHostPort;
}

std::string CConf::getNetworkLocalAddress() const
{
	return m_networkLocalAddress;
}

unsigned short CConf::getNetworkLocalPort() const
{
	return m_networkLocalPort;
}

bool CConf::getNetworkTrace() const
{
	return m_networkTrace;
}
