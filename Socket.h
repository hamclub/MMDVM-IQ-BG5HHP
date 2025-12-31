/*
 *   Copyright (C) 2024,2025 by Jonathan Naylor G4KLX
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

#if !defined(Socket_H)
#define Socket_H

#include "RingBuffer.h"
#include "UDPSocket.h"


class CSocket {
public:
	CSocket();
	~CSocket();

	bool open(const std::string& myAddress, unsigned short myPort, const std::string& hostAddress, unsigned short hostPort);

	bool    available();
	uint8_t read();
	bool    write(const uint8_t* buffer, uint16_t length);

	void close();

	static void startup();
	static void shutdown();

private:
	sockaddr_storage m_address;
	unsigned int     m_addressLength;
	CUDPSocket*      m_socket;
	CRingBuffer<uint8_t> m_buffer;
};

#endif
