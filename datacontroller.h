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

#ifndef DATACONTROLLER_H_
#define DATACONTROLLER_H_

#include <string>
#include "serialdv_export.h"

namespace SerialDV
{

// Must match the 160-sample (20ms @ 8kHz) framing baked into the fixed
// DV3000_AUDIO_HEADER template's declared payload length (0x0142 = 322 =
// 2 fixed header bytes + 160*2 audio bytes) -- every caller in this
// codebase also sizes its PCM buffers as MBE_AUDIO_BLOCK_SIZE (160), so a
// mismatch here both desyncs the AMBE3000's wire framing (encodeIn sends
// more audio bytes than the header promises) and overruns the caller's
// buffer (decodeOut writes this many samples into it).
const unsigned int MBE_AUDIO_BLOCK_SIZE_INTERNAL  = 160U;
const unsigned int MBE_AUDIO_BLOCK_BYTES_INTERNAL = MBE_AUDIO_BLOCK_SIZE_INTERNAL * 2U;
const unsigned int MBE_FRAME_MAX_LENGTH_BYTES_INTERNAL = 24U;

enum SERIAL_SPEED {
	SERIAL_NONE   = 0,
    SERIAL_1200   = 1200,
    SERIAL_2400   = 2400,
    SERIAL_4800   = 4800,
    SERIAL_9600   = 9600,
    SERIAL_19200  = 19200,
    SERIAL_38400  = 38400,
    SERIAL_76800  = 76800,
    SERIAL_115200 = 115200,
    SERIAL_230400 = 230400,
    SERIAL_460800 = 460800
};

class SERIALDV_API DataController {
public:
    DataController();
    virtual ~DataController();

    virtual bool open(const std::string& device, SERIAL_SPEED speed) = 0;

    virtual bool initResponse() = 0;
    virtual int  read(unsigned char* buffer, unsigned int lengthInBytes) = 0;
    virtual int  write(const unsigned char* buffer, unsigned int lengthInBytes) = 0;

    virtual void closeIt() = 0;

#ifdef __WINDOWS__
    static const unsigned int BUFFER_LENGTH = 1000U;
#else
    static const unsigned int BUFFER_LENGTH = 400U;
#endif
};

} // namespace SerialDV

#endif // DATACONTROLLER_H_
