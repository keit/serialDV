///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016 Edouard Griffiths, F4EXB.                                  //
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

#include <chrono>
#include <thread>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <stdint.h>

#ifdef __APPLE__
#include "dummydatacontroller.h"
#else
#include "udpdatacontroller.h"
#include "serialdatacontroller.h"
#endif
#include "dvcontroller.h"

namespace SerialDV
{

DVController::DVController() :
        m_serial(nullptr),
        m_open(false),
        m_currentRate(DVRateNone),
        m_currentGainIn(0),
        m_currentGainOut(0),
        m_currentNbMbeBits(72),
        m_currentNbMbeBytes(9)
{
    m_littleEndian = isLittleEndian();
}

DVController::~DVController()
{
    if (m_serial) {
        delete m_serial;
    }
}

bool DVController::open(const std::string& device, bool halfSpeed)
{
    m_open = false;

#ifdef __APPLE__
    m_serial = new DummyDataController();
#else
    if (device.find(':') != std::string::npos) {
        m_serial = new UDPDataController();
    } else {
        m_serial = new SerialDataController();
    }
#endif

    bool res = m_serial->open(device, halfSpeed ? SERIAL_230400 : SERIAL_460800);

    if (!res) {
        return false;
    }

    m_serial->write(DV3000_REQ_PRODID, DV3000_REQ_PRODID_LEN);

    unsigned char buffer[DataController::BUFFER_LENGTH];
    RESP_TYPE type = getResponse(buffer, DataController::BUFFER_LENGTH);

    if (type == RESP_ERROR)
    {
        fprintf(stderr, "DVController::open: serial device error\n");
        m_serial->closeIt();
        return false;
    }
    else if (type == RESP_NAME)
    {
        std::string name((char *) &buffer[5]);
        fprintf(stderr, "DVController::open: DV3000 chip identified as: %s\n", name.c_str());
        m_open = true;
        return true;
    }
    else
    {
        fprintf(stderr, "DVController::open: response mismatch\n");
        m_serial->closeIt();
        return false;
    }
}

void DVController::close()
{
    m_serial->closeIt();
    m_open = false;
}

bool DVController::encode(const short *audioFrame, unsigned char *mbeFrame, DVRate rate, int gain)
{
	if (!m_open) {
		return false;
	}

	if (rate != m_currentRate)
	{
	    setRate(rate);
	    m_currentRate = rate;
	}

	if (gain != m_currentGainIn)
	{
	    setGain(gain, m_currentGainOut);
	    m_currentGainIn = gain;
	}
	encodeIn(audioFrame, MBE_AUDIO_BLOCK_SIZE_INTERNAL);
	return encodeOut(mbeFrame, m_currentNbMbeBytes);
}


bool DVController::decode(short *audioFrame, const unsigned char *mbeFrame, DVRate rate, int gain)
{
	if (!m_open) {
		return false;
	}

    if (rate != m_currentRate)
    {
        setRate(rate);
        m_currentRate = rate;
    }

    if (gain != m_currentGainOut)
    {
        setGain(m_currentGainIn, gain);
        m_currentGainOut = gain;
    }

	decodeIn(mbeFrame, m_currentNbMbeBits, m_currentNbMbeBytes);
	return decodeOut(audioFrame, MBE_AUDIO_BLOCK_SIZE_INTERNAL);
}

unsigned short DVController::getNbMbeBytes(DVRate mbeRate)
{
    switch (mbeRate)
    {
    case DVRateNone:
        return 0;
    case DVRate3600x2400:
        return 9;
        break;
    case DVRate3600x2450:
        return 9;
        break;
    case DVRate7200x4400:
        return 18;
        break;
    case DVRate2450:
        return 7;
        break;
    case DVRate4400:
        return 11;
        break;
    case DVRate2200:
        return 6;
        break;
    case DVRate3000:
        return 8;
        break;
    case DVRate6400:
        return 16;
        break;
    case DVRate7200:
        return 18;
        break;
    case DVRate8000:
        return 20;
        break;
    case DVRate9600:
        return 24;
        break;
    default:
        return 0;
    }
}

unsigned char DVController::getNbMbeBits(DVRate mbeRate)
{
    switch (mbeRate)
    {
    case DVRateNone:
        return 0;
    case DVRate3600x2400:
        return 72;
        break;
    case DVRate3600x2450:
        return 72;
        break;
    case DVRate7200x4400:
        return 144;
        break;
    case DVRate2450:
        return 49;
        break;
    case DVRate4400:
        return 88;
        break;
    case DVRate2200:
        return 44;
        break;
    case DVRate3000:
        return 60;
        break;
    case DVRate6400:
        return 128;
        break;
    case DVRate7200:
        return 144;
        break;
    case DVRate8000:
        return 160;
        break;
    case DVRate9600:
        return 192;
        break;
    default:
        return 0;
    }
}

bool DVController::setGain(signed char dBGainIn, signed char dBGainOut)
{
    if (!m_open) {
        return false;
    }

    if (dBGainIn < -90) {
        dBGainIn = -90;
    } else if (dBGainIn > 90) {
        dBGainIn = 90;
    }

    if (dBGainOut < -90) {
        dBGainOut = -90;
    } else if (dBGainOut > 90) {
        dBGainOut = 90;
    }

    unsigned char buffer[DV3000_REQ_GAIN_LEN + 2];
    ::memcpy(buffer, DV3000_REQ_GAIN, DV3000_REQ_GAIN_LEN);

    buffer[DV3000_REQ_GAIN_LEN]   = dBGainIn;
    buffer[DV3000_REQ_GAIN_LEN+1] = dBGainOut;

    m_serial->write(buffer, DV3000_REQ_GAIN_LEN + 2);
    RESP_TYPE type = getResponse(buffer, DataController::BUFFER_LENGTH);

    if (type == RESP_ERROR)
    {
        fprintf(stderr, "DVController::setGain: serial device error\n");
        return false;
    }
    else if (type == RESP_GAIN)
    {
        fprintf(stderr, "DVController::setGain: in: %d dB out: %d dB: OK\n", (int) dBGainIn, (int) dBGainOut);
        return true;
    }
    else
    {
        fprintf(stderr, "DVController::setGain: response mismatch\n");
        return false;
    }
}

void DVController::encodeIn(const short* audio, unsigned int length)
{   
    (void) length;
    assert(audio != 0);
    assert(length == MBE_AUDIO_BLOCK_SIZE_INTERNAL);

    // TODO: optimization with fixed initialization of the audio header
    unsigned char buffer[DV3000_AUDIO_HEADER_LEN + MBE_AUDIO_BLOCK_BYTES_INTERNAL];

    ::memcpy(buffer, DV3000_AUDIO_HEADER, DV3000_AUDIO_HEADER_LEN);

    uint8_t* q = (uint8_t*) (buffer + DV3000_AUDIO_HEADER_LEN);

    for (unsigned int i = 0; i < MBE_AUDIO_BLOCK_SIZE_INTERNAL; i++, q += 2U)
    {
        q[0U] = (audio[i] & 0xFF00) >> 8;
        q[1U] = (audio[i] & 0x00FF) >> 0;
    }

    m_serial->write(buffer, DV3000_AUDIO_HEADER_LEN + MBE_AUDIO_BLOCK_BYTES_INTERNAL);
}

bool DVController::encodeOut(unsigned char* ambe, unsigned int length)
{
    assert(ambe != 0);
    assert(length == m_currentNbMbeBytes);

    unsigned char buffer[DataController::BUFFER_LENGTH];
    RESP_TYPE type = getResponse(buffer, DataController::BUFFER_LENGTH);

    if (type != RESP_AMBE)
    {
        fprintf(stderr, "DVController::encodeOut: error\n");
        return false;
    }

    ::memcpy(ambe, buffer + DV3000_AMBE_HEADER_LEN, length);

    return true;
}

void DVController::decodeIn(const unsigned char* ambe, unsigned char nbBits, unsigned short nbBytes)
{
    assert(ambe != 0);
    assert(nbBytes == m_currentNbMbeBytes);
    unsigned short length = nbBytes + 2;
    unsigned char *lengthPtr = (unsigned char *) &length;

    unsigned char buffer[DV3000_AMBE_HEADER_LEN + MBE_FRAME_MAX_LENGTH_BYTES_INTERNAL];
    ::memcpy(buffer, DV3000_AMBE_HEADER, DV3000_AMBE_HEADER_LEN);
    ::memcpy(buffer + DV3000_AMBE_HEADER_LEN, ambe, nbBytes);

    if (m_littleEndian)
    {
        ::memcpy(&buffer[1], &lengthPtr[1], 1); // set header length field with little endian byte order
        ::memcpy(&buffer[2], &lengthPtr[0], 1); // set header length field with little endian byte order
    }
    else
    {
        ::memcpy(&buffer[1], &lengthPtr[0], 1); // set header length field with big endian byte order
        ::memcpy(&buffer[2], &lengthPtr[1], 1); // set header length field with big endian byte order
    }

    ::memcpy(&buffer[5], &nbBits, 1); // set CHAND number of bits

    m_serial->write(buffer, DV3000_AMBE_HEADER_LEN + nbBytes);
}

bool DVController::decodeOut(short* audio, unsigned int length)
{
    (void) length;
    assert(audio != 0);
    assert(length == MBE_AUDIO_BLOCK_SIZE_INTERNAL);

    unsigned char buffer[DataController::BUFFER_LENGTH];
    RESP_TYPE type = getResponse(buffer, DataController::BUFFER_LENGTH);

    if (type != RESP_AUDIO)
    {
        fprintf(stderr, "DVController::decodeOut: error\n");
        return false;
    }

    uint8_t* q = (uint8_t*) (buffer + DV3000_AUDIO_HEADER_LEN);

    for (unsigned int i = 0U; i < MBE_AUDIO_BLOCK_SIZE_INTERNAL; i++, q += 2U)
    {
        short word = (q[0] << 8) | (q[1U] << 0);
        audio[i] = word;
    }

    return true;
}

bool DVController::setRate(DVRate rate)
{
    fprintf(stderr, "DVController::setRate begin \n");
    if (!m_open) {
        return false;
    }

    if (rate == DVRateNone) {
        return true;
    }

    const unsigned char *ratepStr;

    switch(rate)
    {
    case DVRateNone:
        return true;
    case DVRate3600x2400:
        ratepStr = DV3000_REQ_3600X2400_RATEP;
        m_currentNbMbeBits = 72;
        m_currentNbMbeBytes = 9;
        break;
    case DVRate3600x2450:
        ratepStr = DV3000_REQ_3600X2450_RATEP;
        m_currentNbMbeBits = 72;
        m_currentNbMbeBytes = 9;
        break;
    case DVRate7200x4400:
        ratepStr = DV3000_REQ_7200X4400_3_RATEP; // AMBE 3000 version
        m_currentNbMbeBits = 144;
        m_currentNbMbeBytes = 18;
        break;
    case DVRate2450:
        ratepStr = DV3000_REQ_2450_RATEP;
        m_currentNbMbeBits = 49;
        m_currentNbMbeBytes = 7;
        break;
    case DVRate4400:
        ratepStr = DV3000_REQ_4400_RATEP;
        m_currentNbMbeBits = 88;
        m_currentNbMbeBytes = 11;
        break;
    case DVRate2200:
        ratepStr = DV3000_REQ_2200_RATEP;
        m_currentNbMbeBits = 44;
        m_currentNbMbeBytes = 6;
        break;
    case DVRate3000:
        ratepStr = DV3000_REQ_3000_RATEP;
        m_currentNbMbeBits = 60;
        m_currentNbMbeBytes = 8;
        break;
    case DVRate6400:
        ratepStr = DV3000_REQ_6400_RATEP;
        m_currentNbMbeBits = 128;
        m_currentNbMbeBytes = 16;
        break;
    case DVRate7200:
        ratepStr = DV3000_REQ_7200_RATEP;
        m_currentNbMbeBits = 144;
        m_currentNbMbeBytes = 18;
        break;
    case DVRate8000:
        ratepStr = DV3000_REQ_8000_RATEP;
        m_currentNbMbeBits = 160;
        m_currentNbMbeBytes = 20;
        break;
    case DVRate9600:
        ratepStr = DV3000_REQ_9600_RATEP;
        m_currentNbMbeBits = 192;
        m_currentNbMbeBytes = 24;
        break;
    default:
        return true;
    }

    m_serial->write(ratepStr, DV3000_REQ_RATEP_LEN);

    unsigned char buffer[DataController::BUFFER_LENGTH];
    RESP_TYPE type = getResponse(buffer, DataController::BUFFER_LENGTH);

    if (type == RESP_ERROR)
    {
        fprintf(stderr, "DVController::setRate: serial device error\n");
        return false;
    }
    else if (type == RESP_RATEP)
    {
        fprintf(stderr, "DVController::setRate (%d): OK\n", (int) rate);
        return true;
    }
    else
    {
        fprintf(stderr, "DVController::setRate: response mismatch\n");
        return false;
    }
    fprintf(stderr, "DVController::setRate begin \n");

}

DVController::RESP_TYPE DVController::getResponse(unsigned char* buffer, unsigned int length)
{
    (void) length;
    assert(buffer != 0);
    assert(length >= DataController::BUFFER_LENGTH);

    if (!m_serial->initResponse())
    {
        fprintf(stderr, "DVController::getResponse: cannot get response\n");
        return RESP_ERROR;
    }

    // Retries used to be capped at a fixed 5 attempts per stage regardless
    // of whether the read was actively progressing. That's enough for the
    // small control responses (PRODID/RATEP/GAIN, a handful of bytes) but
    // not for a full 322-byte AUDIO payload: the FTDI driver hands bytes to
    // the kernel in whatever chunks USB polling happened to batch up (with
    // the 1ms latency_timer, often a few dozen bytes per read()), so
    // assembling 322 bytes can legitimately take more than 5 reads even
    // though the chip is actively, correctly sending the whole time.
    // Cutting the read off mid-transfer doesn't stop the chip from
    // finishing it anyway -- those never-consumed trailing bytes then show
    // up as garbage corrupting the *next* exchange's start-byte search
    // (confirmed by instrumenting this: a payload timeout at 221/322 bytes
    // was immediately followed by stray non-start bytes on the next call).
    // So retry as long as each attempt is still making forward progress,
    // and only give up after several *consecutive* attempts return
    // nothing at all -- that keeps the same fast failure behaviour for a
    // genuinely unresponsive chip while giving a slow-but-live transfer as
    // many reads as it actually needs.
    const int maxNoProgressAttempts = 5;

    bool found = false;
    int packetLength, offset;
    unsigned char packetType;
    int noProgress = 0;
    // A stray (non-start) byte is evidence the stream is live, just not
    // aligned yet, so it resets the no-progress counter below rather than
    // counting against it -- but that alone would let truly endless line
    // noise (never containing a real start byte) spin forever, so this is
    // an independent hard cap on total bytes scanned regardless of
    // progress: generous enough to skip past any leftover backlog, but finite.
    const int maxStrayBytes = 256;
    int strayBytes = 0;

    while (noProgress < maxNoProgressAttempts && strayBytes < maxStrayBytes)
    {
        int len1 = m_serial->read(buffer, 1U);

        if (len1 < 0)
        {
            fprintf(stderr, "DVController::getResponse: Error (start byte)\n");
            return RESP_ERROR;
        }
        else if ((len1 == 1) && (buffer[0U] == DV3000_START_BYTE))
        {
            found = true;
            break;
        }
        else if (len1 == 1)
        {
            noProgress = 0;
            strayBytes++;
        }
        else
        {
            noProgress++;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    if (!found)
    {
        fprintf(stderr, "DVController::getResponse: Timeout (start byte)\n");
        return RESP_ERROR;
    }

    packetLength = 3;
    offset = 0;
    found = false;
    noProgress = 0;

    while (noProgress < maxNoProgressAttempts)
    {
        int len1 = m_serial->read(&buffer[1 + offset], packetLength - offset);

        if (len1 < 0)
        {
            fprintf(stderr, "DVController::getResponse: Error (packet header at %d)\n", offset);
            return RESP_ERROR;
        }
        else if (offset + len1 == packetLength)
        {
            found = true;
            break;
        }
        else if (len1 > 0)
        {
            offset += len1;
            noProgress = 0;
        }
        else
        {
            noProgress++;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    if (!found)
    {
        fprintf(stderr, "DVController::getResponse: Timeout (packet header), got %d/%d bytes: %02x %02x %02x %02x\n",
                offset, packetLength, buffer[0], buffer[1], buffer[2], buffer[3]);
        return RESP_ERROR;
    }

    packetLength = buffer[1] * 256 + buffer[2];
    packetType = buffer[3];
    offset = 0;
    found = false;
    noProgress = 0;

    if (4 + packetLength > (int) length)
    {
        fprintf(stderr, "DVController::getResponse: packet length %d exceeds buffer, dropping\n", packetLength);
        return RESP_ERROR;
    }

    while (noProgress < maxNoProgressAttempts)
    {
        int len1 = m_serial->read(&buffer[4 + offset], packetLength - offset);

        if (len1 < 0)
        {
            fprintf(stderr, "DVController::getResponse: Error (packet payload at %d)\n", offset);
            return RESP_ERROR;
        }
        else if (offset + len1 == packetLength)
        {
            found = true;
            break;
        }
        else if (len1 > 0)
        {
            offset += len1;
            noProgress = 0;
        }
        else
        {
            noProgress++;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    if (!found) {
        fprintf(stderr, "DVController::getResponse: Timeout (packet payload), got %d/%d bytes, "
                        "header was: start=%02x len=%02x%02x type=%02x\n",
                offset, packetLength, buffer[0], buffer[1], buffer[2], buffer[3]);
        return RESP_ERROR;
    }

    //fprintf(stderr, "DVController::getResponse: packet type %02x\n", packetType);

    if (packetType == DV3000_TYPE_AUDIO)
    {
        return RESP_AUDIO;
    }
    else if (packetType == DV3000_TYPE_AMBE)
    {
        return RESP_AMBE;
    }
    else if (packetType == DV3000_TYPE_CONTROL) // check the field type buffer[4]
    {
        //fprintf(stderr, "DVController::getResponse: field type %02x\n", buffer[4]);

        if (buffer[4] == DV3000_CONTROL_PRODID)
        {
            return RESP_NAME;
        }
        else if (buffer[4] == DV3000_CONTROL_RATEP)
        {
            return RESP_RATEP;
        }
        else if (buffer[4] == DV3000_CONTROL_GAIN)
        {
            return RESP_GAIN;
        }
        else if (buffer[4] == DV3000_CONTROL_READY)
        {
            return RESP_UNKNOWN;
        }
        else
        {
            return RESP_UNKNOWN;
        }
    }
    else
    {
        return RESP_UNKNOWN;
    }
}

} // namespace SerialDV

