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

#ifndef UDPDATACONTROLLER_H_
#define UDPDATACONTROLLER_H_

#include <string>
#include "datacontroller.h"

namespace SerialDV
{

// Talks raw DV3000 packets to a remote AMBE3000 over UDP -- e.g. PA7LIM's
// AMBEServer 3000 or the DVSwitch AMBEserver it descends from, which pass each
// datagram straight to the chip's serial port and send the chip's reply back
// to the requester as one datagram. One request datagram in, one response
// datagram out, so DVController's byte-oriented read() loop works unchanged
// on top of a buffered datagram.
//
// The device string is "host:port" (hostname or IPv4 dotted quad). The local
// socket uses an ephemeral port and is connect()ed to the server, so it works
// against a server on the same machine and ignores datagrams from anyone else.
class SERIALDV_API UDPDataController : public DataController {
public:
    UDPDataController();
    virtual ~UDPDataController();

    virtual bool open(const std::string& hostAndPort, SERIAL_SPEED speed);

    virtual bool initResponse();
    virtual int  read(unsigned char* buffer, unsigned int lengthInBytes);
    virtual int  write(const unsigned char* buffer, unsigned int lengthInBytes);

    virtual void closeIt();

    // A remote AMBEServer passes our bytes straight to the chip's serial port.
    // If a request is ever cut short on that last hop, the chip keeps waiting
    // for the rest of the packet it thinks it is receiving and swallows every
    // following request as payload, so nothing is answered again -- until
    // enough bytes arrive to finish it. Sending a full-size silent audio
    // frame does that, and the reply to it shows the link is back.
    virtual bool resync();
    virtual void discardPending() { drainPending(); }

private:
    // Discards any datagrams already waiting: a reply that arrived after its
    // request timed out would otherwise be taken as the *next* request's
    // reply. The UDP counterpart of the serial input flush before write().
    void drainPending();

    int m_sockFd;
    unsigned char m_responseBuffer[2000];
    int m_responseSize;
    int m_responseIndex;
    // The first response after open() (the product-ID query) also covers ARP
    // resolution and any server-side wake-up, so it gets a longer wait than
    // the steady-state per-frame exchanges.
    bool m_firstResponse;
};

} // namespace SerialDV

#endif // UDPDATACONTROLLER_H_
