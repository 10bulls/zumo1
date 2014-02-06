#include <stdio.h>    // For sprintf
#include <string.h>   // For strlen
/* Note: Not using C++ headers (cstdio, algorithm etc.) because
 *       at the moment, they don't exist on AVR / Arduino.
 */

#include <Arduino.h>

#include "zmodem.h"

#ifdef __AVR__
# include <util/crc16.h>
#endif

namespace
{
    /*--------------------------------------------------*/
    /* CRC-16 and CRC-32 functions                      */
    /* Because of memory size limits, not using tables  */
    /*--------------------------------------------------*/

#ifdef __AVR__ /* AVR-LIBC has an optimized CRC16 function, and we will use it. */
    const auto CRC16 = _crc_xmodem_update;
#else
    uint16_t CRC16(uint16_t crc, uint8_t c) __attribute__((noinline));
    uint16_t CRC16(uint16_t crc, uint8_t c)
    {
        for(crc ^= (c << 8), c = 8; c-- > 0; )
            crc = (crc << 1) ^ (0x1021 & ~((crc >> 15)-1));
        return crc;
    }
#endif

    uint32_t CRC32(uint32_t crc, uint8_t c) __attribute__((noinline));
    uint32_t CRC32(uint32_t crc, uint8_t c)
    {
        for(crc ^= c,  c = 8; c-- > 0; )
            crc = (crc >> 1) ^ (0xEDB88320UL & ~((crc & 1l)-1l));
        return crc;
    }

    /* List of hexadecimal digits */
    const char hexdigits[16] =
        {'0','1','2','3','4','5','6','7', '8','9','a','b','c','d','e','f'};

    /* min() and max() functions */
    #undef min
    template<typename T,typename U> auto min(const T& a, const U& b)
                                     -> decltype(a < b ? a : b)
        { return (a<b ? a : b); }
    #undef max
    template<typename T,typename U> auto max(const T& a, const U& b)
                                     -> decltype(a > b ? a : b)
        { return (a>b ? a : b); }

    // Block size limits
    static const unsigned max_block_size = 8192; // GNU lrzsz limit is 8192.
    static const unsigned min_block_size = 1;
}

bool ZMODEM::Send(const char* filename, uint32_t file_length)
{
    // Begin ZMODEM process
    SendByte( 'r', 'z', CR ); // Send "rz\r" to prepare the receiver.

    // This encapsulates a ZMODEM message.
    struct ZMessage
    {
        short type;       // Type (0-19, or -1 indicating an error)
        uint8_t data[4];  // Parameters (4 bytes or one long integer)

        ZMessage(): type(ERROR){}

        ZMessage(short t, uint32_t p = 0)
            : type(t), data{ uint8_t(p>>0),  uint8_t(p>>8),
                             uint8_t(p>>16), uint8_t(p>>24) } {}

        uint32_t Value() const
        {
            return data[0]*0x00001ul + data[1]*0x0000100ul
                 + data[2]*0x10000ul + data[3]*0x1000000ul;
        }
    };

    // Sending state variables
    struct
    {
        uint32_t crc;
        uint8_t  mode:7;       // ZBIN/ZHEX/ZBIN32
        bool     frame_open:1;
    } sent = {0, ZHEX, false};

    // Utility function for sending one byte and updating the CRC.
    auto SendByteEncoded = [&](uint8_t v) -> void
    {
        // Update the CRC
        if(sent.mode == ZBIN32)
            sent.crc = CRC32(sent.crc, v);
        else
            sent.crc = CRC16(sent.crc, v);

        if(sent.mode == ZHEX) // Send as two hexadecimal digits
        {
            SendByte( hexdigits[v>>4], hexdigits[v&0xF] );
        }
        else // Send raw (binary)
        {
            uint8_t v7 = v & 0x7F; // v clamped to 7 bits

            // Certain bytes must be escaped though.
            if(v7 == DLE || v7 == XON || v7 == XOFF || v7 == ZDLE)
                SendByte( ZDLE, v ^ 0x40);
            else
                SendByte( v );
        }
    };
    // Utility function for sending the CRC.
    auto SendCRC = [&]() -> void
    {
        auto c = sent.crc;
        if(sent.mode == ZBIN32)
            for(int a=0; a<32; a+=8) SendByteEncoded(~c >> a);
        else
            for(int a=8; a>=0; a-=8) SendByteEncoded(c >> a);
    };
    // Beginning and ending a data block
    auto SendDataBegin = [&]() -> void
    {
        sent.crc = (sent.mode == ZBIN32 ? ~uint32_t(0) : 0);
    };
    auto SendDataEnd = [&](uint8_t frameend) -> void
    {
        SendByte( ZDLE );
        SendByteEncoded(frameend);
        SendCRC();
        if(frameend == ZCRCW || frameend == ZCRCQ) SendByte( XON );
        sent.frame_open = (frameend == ZCRCG || frameend == ZCRCQ);
    };
    // Send ZMODEM message, (in either binary or hex)
    auto SendMessage = [&](const ZMessage& msg, uint8_t mode) -> void
    {
        // Before a new message can be sent, a possibly open previous
        // message must first be closed.
        if(sent.frame_open) { SendDataBegin(); SendDataEnd(ZCRCE); }

        sent.mode = mode;
        SendDataBegin();
        SendByte(ZPAD, ZPAD, ZDLE, sent.mode);
        SendByteEncoded(msg.type);
        for(unsigned a=0; a<4; ++a) SendByteEncoded(msg.data[a]);
        SendCRC();

        if(sent.mode == ZHEX) SendByte( CR, LF|0x80, XON );
    };

    // Receiving state variables
    struct ReceivedType
    {
        ZMessage message;
        uint16_t crc;
        unsigned garbage;      // Number of garbage characters received
        uint8_t incomplete:4;  // Current state machine phase. 0 = message received.
        uint8_t cans:4;        // Number of ZDLE received (5 to abort)
        uint8_t byte, n:4,m:4; // Miscellaneous
        uint8_t mode;          // Type of message received (ZBIN or ZHEX)

        ReceivedType(): garbage(0),incomplete(1),cans(0), byte(),n(),m(0),mode() { }

        void Reset() { *this = ReceivedType(); }
        void Finish(short type) { message.type = type; incomplete = 0; }

        // Deal with a received byte (or with an I/O error)
        void Update(const short c)
        {
            // An I/O error terminates the message
            if(c < 0) return Finish(c);

            // c clamped to 7 bits
            const uint8_t c7 = c & 0x7F;

            // Ignore XON/XOFF in the data stream
            if(c7 == XON || c7 == XOFF) return;

            // At any time, detect a sequence of 5*CAN as an abort signal.
            if(c7 == CAN) { if(++cans >= 5) return Finish(ZCAN); }
            else          { cans = 0; }

            switch(incomplete)
            {
                // Detect a ZMODEM message: ZPAD ZDLE ZBIN|ZHEX
                for(;;)  // m: 0=waits for ZPAD, 1=needs ZDLE, 2=needs ZBIN/ZHEX
                {
                    return void(incomplete=1);         // Wait for next character
            case 1: if(c7 == ZPAD) m = 1;              // Ignore repeated ZPADs
                    else if(m==1 && c7 == ZDLE) m = 2; // ZDLE following a ZPAD
                    else if(m==2 && (c7 == ZBIN || c7 == ZHEX)) break;
                    else m = 0;                        // Anything else: garbage
                    // Too much garbage = Cancel transmission
                    if(!m && ++garbage >= 1000) return Finish(ZCAN);
                }

                // Message prefix received successfully. Now read the actual message.
                //
                mode = c7;

                // We will read: Type[1 byte] Data[4 byte] Crc[2 bytes] = 7 bytes total
                for(crc = n = 0; n < 7; ++n)
                {
                    if(mode == ZHEX) // Hexadecimal message. Read two hexadecimal digits.
                    {
                        for(byte = 0, m = 0; m < 2; ++m)
                        {
                            // Read byte (c)
                            return void(incomplete=2);
            case 2:     // Returns here with the new byte (c, and c7)
                            /**/ if(c7 >= '0' && c7 <= '9') byte = byte*16 + c7 + 0  - '0';
                            else if(c7 >= 'a' && c7 <= 'f') byte = byte*16 + c7 + 10 - 'a';
                            else return Finish(ERROR);
                        }
                    }
                    else // Binary message. Read raw byte and check for ZMODEM escape encoding.
                    {
                        for(m=0; ; m=1) { return void(incomplete=4);
            case 4:                       if(c!=ZDLE) break; }
                        // Skip any ZDLE bytes, but mark whether ZDLE was encountered.
                        if(!m) byte = c; // No ZDLE? Take byte verbatim.
                        else if(c == ZRUB0) byte = 0x7F; // ZDLE+ZRUB0 = 0x7F
                        else if(c == ZRUB1) byte = 0xFF; // ZDLE+ZRUB1 = 0xFF
                        else if((c & 0x60) == 0x40) byte = c ^ 0x40; // And so on
                        else return Finish(ERROR); // bad escape sequence
                    }
                    // Update checksum
                    crc = CRC16(crc, byte);
                    // Store the non-checksum data into the message
                    if(n == 0) message.type = byte;
                    else if(n <= 4) message.data[n - 1] = byte;
                }
                // Verify checksum
                if (crc != 0) return Finish(ERROR);

                // (Hex messages only): Throw away possible cr/lf
                if (mode == ZHEX)
                {
                    m=0;
                    do { return void(incomplete=3);
            case 3:   ;} while(c7 != LF || ++m==2);
                }

                // Message received successfully.
                return Finish(message.type);
            }
        }
    } received;

    // Begin file transmission.
    // Request receiver to assert their presence as a ZMODEM client.
    SendMessage(ZRQINIT, ZHEX);

    // Transfer state variables
    bool     result = false;
    bool     done   = false;
    unsigned block_size = 512;
    unsigned char success = 0;
    uint32_t position           = 0;
    uint32_t last_good_position = 0;
    uint32_t file_crc = 0;
    bool     file_crc_known = false;
    bool     zfile_hdr_sent = false;
    bool     force_new_zdata = true;
    bool     NeedsResponse = true; // Initially needs response to ZRQINIT
    uint8_t  frame_format = ZBIN;
    uint8_t  HangCounter = 0;

    while(!done && Carrier())
    {
        // If we need to read incoming data, make it so.
        while(received.incomplete && (CharAvail() || NeedsResponse))
            received.Update( GetByte(20) );

        // If there is a response, deal with it.
        if(!received.incomplete)
        {
            if(++HangCounter >= 16)
            {
                // If we're handling too many messages in a row
                // without really getting anything done, assume
                // the peer is deliberately stalling us, and quit.
                done = true;
                break;
            }
            switch(received.message.type)
            {
                case ZRINIT: // Receiver is ready to receive a new file.
                    // If the receiver can do 32-bit CRC, enable that feature
                    if(received.message.Value() & 0x20000000ul) frame_format = ZBIN32;

                    // Did we not already send a file?
                    if(position < file_length)
                    {
                        /* Send ZFILE message to receiver, telling what we are sending.
                         * The data section of the message contains two parameters,
                         * separated by a '\0' character. The first parameter is
                         * the file name, and the second parameter contains some
                         * numeric values, the first of which is the file size.
                         */
                        char info[32];
                        unsigned l = strlen(filename);
//!                        unsigned m = sprintf(info, "%lu 0 0644", (unsigned long)file_length);
						unsigned m = snprintf(info, sizeof(info), "%lu 0 0644", (unsigned long)file_length);

//!						Serial.println("HERE");
//!						Serial.println(info);

                        // Send ZFILE message with ZF1_ZMCRC option
                        if(!zfile_hdr_sent)
                        {
                            // But don't send it twice. It may sometimes happen
                            // that we get a duplicate ZRINIT (if rz is started
                            // before we sent ZRQINIT).
                            // If we send it twice, we get an ugly cascade
                            // of mis-synchronized request-replies.
                            //
                            SendMessage( {ZFILE, 0x00020000ul }, frame_format );
                            SendDataBegin();
                            for(unsigned a=0; a<=l; ++a) SendByteEncoded(filename[a]);
                            for(unsigned a=0; a<m;  ++a) SendByteEncoded(info[a]);
                            SendDataEnd(ZCRCW);
                            zfile_hdr_sent = true;
                        }

                        block_size = max_block_size;
                        NeedsResponse      = true;
                        last_good_position = 0;
                        position           = 0;
                        break;
                    }
                    // We already sent a file. Finish the session. passthru:
                case ZSKIP: // When recipient wants to skip this file
                    SendMessage( ZFIN, ZHEX );
                    NeedsResponse = true;
                    position      = file_length;
                    break;
                case ZFIN:
                    // End transmission. Error if we aren't at the end of the file.
                    result = position == file_length;
                    done   = true;
                    break;
                case ZCHALLENGE: // Echo receiver's challenge number
                    SendMessage( {ZACK, received.message.Value()}, ZHEX );
                    break;
                case ZCOMMAND: // Miscellaneous: They didn't see our ZRQINIT
                    SendMessage( ZRQINIT, ZHEX ); // Resend it
                    NeedsResponse = true;
                    break;
                case ZRPOS:
                    // Begin sending / resend from this position
                    position           = received.message.Value();
                    last_good_position = position;
                    force_new_zdata    = true;
                    NeedsResponse      = false;
                    block_size /= 4;
                    success = 0;
                    break;
                case ZCRC:
                    // Receiver wants to know the CRC for our entire file.
                    if(!file_crc_known || received.message.Value())
                    {
                        // It will take some time, so send the receiver
                        // a status message first.
                        static const char message[] =
                            "Calculating ROM checksum.\r\n";
                        SendMessage( ZSTDERR, ZBIN );
                        SendDataBegin();
                        for(unsigned a=0; message[a]; ++a) SendByteEncoded(message[a]);
                        SendDataEnd(ZCRCE);
                        // GNU lrzsz ignores the message though.

                        // Recalculate the CRC32.
                        if(!received.message.Value()) file_crc_known = true;
                        file_crc = ~uint32_t(0);
                        uint32_t limit = received.message.Value() ? received.message.Value() : file_length;
                        uint8_t b;
                        for(uint32_t a=0; a!=limit; ++a)
                        {
                            GetData(&b, a, 1);
                            file_crc = CRC32(file_crc, b);
                        }
                        file_crc = ~file_crc;
                    }
                    // Send the CRC. Next, the receiver will either
                    // send a ZRPOS to initiate a transfer, or a ZSKIP
                    // indicating that he does not need it.
                    // Wait for that response.
                    SendMessage( {ZCRC, file_crc}, ZHEX );
                    NeedsResponse = true;
                    break;
                case ZACK: // Receiver responds to synchronization request
                    if(received.message.Value() != position)
                    {
                        // Receiver has different position in file
                        // than what we expected. This can happen
                        // if some bytes were dropped in transmission.
                        // CRC-16 does not always detect such errors.
                        //
                        // We have to disagree with the recipient's
                        // file position and seek backwards.
                        SendMessage( {ZNAK, received.message.Value()}, ZHEX );
                        // Unfortunately at least GNU lrzsz ignores
                        // the above measure, but at least we tried.

                        case ZNAK: // Land here also if the receiver disputed some frame we sent.
                        position        = last_good_position;
                        force_new_zdata = true;
                        block_size /= 2;
                        success = 0;
                    }
                    else
                        last_good_position = position;

                    // If this ZACK acknowledged the file end, send EOF notification.
                    if(position == file_length)
                        SendMessage( {ZEOF, file_length}, ZHEX );
                    else // Otherwise continue transferring data
                        NeedsResponse = false;
                    break;
                case TIMEOUT:
                    // Return error, if we needed a response and got none
                    if(!NeedsResponse) break;
                    //passthru
                case RCDO:
                case ZABORT:
                case ZCAN:
                case ZFERR:
                    // Any of these messages / signals are unrecoverable
                    result = false;
                    done   = true;
                    break;
                case ERROR:
                default:
                    // Illegible message. Nag at recipient, but don't abort.
                    SendMessage( {ZNAK, received.message.Value()}, ZHEX );
                    break;
            }

            received.Reset();
            continue;
        }

        // Ok. Send data. Begin with ZDATA message unless it was already sent.
        HangCounter = 0;

        if(!sent.frame_open || force_new_zdata)
            SendMessage( {ZDATA,position}, frame_format );

        // Ensure our block size is within acceptable limits
        block_size = min(max_block_size, max(min_block_size, block_size));

        // Choose an amount of data to transfer
        unsigned size = min(block_size, file_length-position);

        int terminator = ZCRCG; // Continue without waiting for receiver
        if(position + size == file_length)
        {
            // At the end of file, terminate the ZDATA frame
            // and wait for a ZACK, so we will know whether
            // the receiver is at same file position as we.
            terminator    = ZCRCW;
            NeedsResponse = true;
        }
        else if(force_new_zdata)
        {
            // After first block, expect an ZACK, so we will know whether
            // the receiver is at same file position as we.
            // Don't end the frame though.
            terminator    = ZCRCQ;
            NeedsResponse = true;
        }

        force_new_zdata = false;

        SendDataBegin();
        // Read data from the data source and send it, 16 bytes at time.
        static uint8_t FileBuffer[16];
        for(auto end = position + size; position != end; )
        {
            uint8_t fill = min(end-position, sizeof FileBuffer);
            GetData(FileBuffer, position, fill);
            for(uint8_t a=0; a<fill; ++a) SendByteEncoded(FileBuffer[a]);
            position += fill;
        }
        SendDataEnd(terminator);

        // When at least 3 blocks have been transmitted successfully
        // at the current block size, try increasing the block size.
        if(terminator == ZCRCG && ++success >= 3)
        {
            last_good_position = position;
            block_size <<= 1;
            success = 0;
        }
    }

    // If we got an error, send a cancel sequence (8*CAN to be sent).
    if(!result) SendByte( CAN,CAN,CAN,CAN,CAN,CAN,CAN,CAN );

    // End ZMODEM process
    SendByte( 'O', 'O' );
    return result;
}
