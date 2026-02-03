/*
 *   Copyright (C) 2026 by Jonathan Naylor G4KLX
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

#include "ModemConnection.h"

#include "Log.h"

#include <cassert>

// For network transfers
const size_t BUFFER_LENGTH = 500;

CModemConnection::CModemConnection() :
m_serial(nullptr),
m_socket(nullptr),
m_debug(false),
m_address(),
m_addressLength(0),
m_buffer(2000U, "Modem UDP Buffer")
{
}

CModemConnection::~CModemConnection()
{
	delete m_serial;
	delete m_socket;
}

void CModemConnection::setUARTConnection(const std::string& port, unsigned int speed, bool debug)
{
	assert(!port.empty());
	assert(speed > 0U);

	m_debug  = debug;
	m_serial = new CUARTController(port, speed);
}

void CModemConnection::setUDPConnection(const std::string& modemAddress, uint16_t modemPort, const std::string& localAddress, uint16_t localPort, bool debug)
{
	assert(!modemAddress.empty());
	assert(!localAddress.empty());
	assert(modemPort > 0U);
	assert(localPort > 0U);

	int ret = CUDPSocket::lookup(modemAddress, modemPort, m_address, m_addressLength);
	if (ret != 0)
		return;

	m_debug  = debug;
	m_socket = new CUDPSocket(localAddress, localPort);
}

bool CModemConnection::open()
{
	if (m_serial != nullptr)
		return m_serial->open();

	if (m_socket != nullptr)
		return m_socket->open();

	LogError("No connection type specified for the Modem - open");

	return false;
}

uint16_t CModemConnection::read(uint8_t* buffer, uint16_t length)
{
	assert(buffer != nullptr);
	assert(length > 0U);

	if (m_serial != nullptr)
		return m_serial->read(buffer, length);

	if (m_socket != nullptr) {
		unsigned char data[BUFFER_LENGTH];
		sockaddr_storage address;
		unsigned int addressLength;

		// Get any network data that is waiting, and store it
		int ret = m_socket->read(data, BUFFER_LENGTH, address, addressLength);
		if (ret > 0)
			m_buffer.addData(data, ret);

		uint16_t size = m_buffer.dataSize();

		// Is there any socket data?
		if (size == 0U)
			return 0U;

		// Is there enough socket data?
		if (size < length)
			length = size;

		m_buffer.getData(buffer, length);

		return length;
	}

	LogError("No connection type specified for the Modem - read");

	return 0U;
}

int16_t CModemConnection::write(const uint8_t* buffer, uint16_t length)
{
	assert(buffer != nullptr);
	assert(length > 0U);

	if (m_serial != nullptr)
		return m_serial->write(buffer, length);

	if (m_socket != nullptr) {
		bool ret = m_socket->write(buffer, length, m_address, m_addressLength);
		return ret ? length : -1;
	}

	LogError("No connection type specified for the Modem - write");

	return -1;
}

void CModemConnection::close()
{
	m_buffer.clear();

	if (m_serial != nullptr) {
		m_serial->close();
		return;
	}

	if (m_socket != nullptr) {
		m_socket->close();
		return;
	}

	LogError("No connection type specified for the Modem - close");
}
