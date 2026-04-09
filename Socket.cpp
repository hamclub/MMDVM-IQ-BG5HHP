/*
 *   Copyright (C) 2024,2025,2026 by Jonathan Naylor G4KLX
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

#include "Socket.h"

#if !defined(_WIN32) && !defined(_WIN64)
#include <cerrno>
#include <cstring>
#endif

#include "Log.h"

const unsigned int BUFFER_LENGTH = 3000U;

CSocket::CSocket() :
m_address(),
m_addressLength(0U),
m_socket(NULL),
m_buffer(BUFFER_LENGTH, "UDP Socket Buffer")
{
}

CSocket::~CSocket()
{
}

void CSocket::startup()
{
	CUDPSocket::startup();
}

void CSocket::shutdown()
{
	CUDPSocket::shutdown();
}

bool CSocket::open(const std::string& myAddress, unsigned short myPort, const std::string& hostAddress, unsigned short hostPort)
{
	int ret = CUDPSocket::lookup(hostAddress, hostPort, m_address, m_addressLength);
	if (ret != 0)
		return false;

	m_socket = new CUDPSocket(myAddress, myPort);

	return m_socket->open();
}

bool CSocket::available()
{
	uint16_t n = m_buffer.dataSize();
	if (n > 0U)
		return true;

	unsigned char buffer[BUFFER_LENGTH];
	sockaddr_storage address;
	unsigned int length = 0U;
	int m = m_socket->read(buffer, BUFFER_LENGTH, address, length);
	if (m <= 0)
		return false;

	m_buffer.addData(buffer, m);

	return true;
}

uint8_t CSocket::read()
{
	uint8_t c = 0U;

	m_buffer.getData(c);

	return c;
}

bool CSocket::write(const uint8_t* buffer, uint16_t length)
{
	return m_socket->write(buffer, length, m_address, m_addressLength);
}

void CSocket::close()
{
	m_socket->close();
	delete m_socket;
	m_socket = NULL;
}
