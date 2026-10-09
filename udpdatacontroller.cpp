///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Edouard Griffiths, F4EXB.                                  //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>

#ifdef __WINDOWS__
#include <winsock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include "udpdatacontroller.h"

namespace SerialDV
{

namespace
{
const int FIRST_RESPONSE_TIMEOUT_US = 500000;
// Per 20ms audio frame. Far longer than a LAN/WiFi round trip normally takes,
// but bounded: a lost datagram stalls the audio thread this long, no more.
const int RESPONSE_TIMEOUT_US = 100000;
const int RESYNC_TIMEOUT_US = 150000;

#ifdef __WINDOWS__
inline void closeSock(int fd) { closesocket(fd); }
#else
inline void closeSock(int fd) { ::close(fd); }
#endif

// Waits up to timeoutUs for fd to become readable.
bool waitReadable(int fd, int timeoutUs)
{
    fd_set fds;
    struct timeval tv;
    tv.tv_sec  = timeoutUs / 1000000;
    tv.tv_usec = timeoutUs % 1000000;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    return select(fd + 1, &fds, nullptr, nullptr, &tv) > 0 && FD_ISSET(fd, &fds);
}
}

UDPDataController::UDPDataController() :
    m_sockFd(-1),
    m_responseSize(0),
    m_responseIndex(0),
    m_firstResponse(true)
{
#ifdef __WINDOWS__
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
}

UDPDataController::~UDPDataController()
{
    closeIt();
#ifdef __WINDOWS__
    WSACleanup();
#endif
}

bool UDPDataController::open(const std::string& hostAndPort, SERIAL_SPEED speed)
{
    (void) speed;
    closeIt();

    // Split at the last colon: "host:port".
    const size_t colon = hostAndPort.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= hostAndPort.size())
    {
        fprintf(stderr, "UDPDataController::open: expected host:port, got \"%s\"\n", hostAndPort.c_str());
        return false;
    }

    const std::string host = hostAndPort.substr(0, colon);
    const std::string portStr = hostAndPort.substr(colon + 1);
    char *end = nullptr;
    const long port = strtol(portStr.c_str(), &end, 10);
    if (*end != '\0' || port < 1 || port > 65535)
    {
        fprintf(stderr, "UDPDataController::open: not a valid port: \"%s\"\n", portStr.c_str());
        return false;
    }

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    struct addrinfo *res = nullptr;
    const int gai = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &res);
    if (gai != 0 || res == nullptr)
    {
        fprintf(stderr, "UDPDataController::open: cannot resolve \"%s\": %s\n", host.c_str(), gai_strerror(gai));
        return false;
    }

    m_sockFd = (int) socket(res->ai_family, res->ai_socktype, res->ai_protocol);

    // connect() on a datagram socket just fixes the peer: the kernel then
    // picks an ephemeral local port and drops datagrams from anyone else.
    if (m_sockFd < 0 || connect(m_sockFd, res->ai_addr, (int) res->ai_addrlen) < 0)
    {
        fprintf(stderr, "UDPDataController::open: cannot set up socket to %s: %s\n",
                hostAndPort.c_str(), strerror(errno));
        freeaddrinfo(res);
        closeIt();
        return false;
    }

    freeaddrinfo(res);
    m_responseSize = 0;
    m_responseIndex = 0;
    m_firstResponse = true;
    return true;
}

bool UDPDataController::initResponse()
{
    if (m_sockFd < 0) {
        return false;
    }

    const int timeoutUs = m_firstResponse ? FIRST_RESPONSE_TIMEOUT_US : RESPONSE_TIMEOUT_US;
    m_firstResponse = false;
    m_responseSize = 0;
    m_responseIndex = 0;

    if (!waitReadable(m_sockFd, timeoutUs)) {
        return false;
    }

    // <= 0 covers an ICMP "port unreachable" surfacing as ECONNREFUSED when
    // nothing is listening on the server port.
    const int n = (int) recv(m_sockFd, (char *) m_responseBuffer, sizeof(m_responseBuffer), 0);
    m_responseSize = n > 0 ? n : 0;
    return m_responseSize > 0;
}

int UDPDataController::read(unsigned char* buffer, unsigned int lengthInBytes)
{
    const int remain = m_responseSize - m_responseIndex;

    if (remain <= 0) {
        return 0;
    }

    const int n = std::min<int>(remain, (int) lengthInBytes);
    std::copy(m_responseBuffer + m_responseIndex, m_responseBuffer + m_responseIndex + n, buffer);
    m_responseIndex += n;
    return n;
}

int UDPDataController::write(const unsigned char* buffer, unsigned int lengthInBytes)
{
    if (m_sockFd < 0) {
        return -1;
    }

    if (!m_keepPendingReplies) {
        drainPending();
    }
    return (int) send(m_sockFd, (const char *) buffer, lengthInBytes, 0);
}

bool UDPDataController::resync()
{
    if (m_sockFd < 0) {
        return false;
    }

    // A silent AUDIO packet, the largest request there is (start 0x61,
    // length 0x0142, type 0x02, then the 0x0200a0 field header and 160
    // samples), so it is enough to complete any packet the chip is midway
    // through. Its own reply is discarded.
    unsigned char pkt[326] = {0x61, 0x01, 0x42, 0x02, 0x00, 0xa0};

    for (int attempt = 0; attempt < 3; attempt++)
    {
        drainPending();

        if (send(m_sockFd, (const char *) pkt, sizeof(pkt), 0) < 0) {
            return false;
        }

        if (waitReadable(m_sockFd, RESYNC_TIMEOUT_US))
        {
            // Let any further replies to the bytes above arrive, then drop
            // them so the retried request sees only its own.
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            drainPending();
            return true;
        }
    }

    return false;
}

void UDPDataController::drainPending()
{
    unsigned char scratch[2000];

    while (waitReadable(m_sockFd, 0)) {
        if (recv(m_sockFd, (char *) scratch, sizeof(scratch), 0) < 0) {
            break; // e.g. ECONNREFUSED from an earlier send; nothing more to discard
        }
    }
}

void UDPDataController::closeIt()
{
    if (m_sockFd >= 0)
    {
        closeSock(m_sockFd);
        m_sockFd = -1;
    }
}

} // namespace SerialDV
