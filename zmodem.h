#include <inttypes.h> // For uint8_t and so on

/* ZMODEM serial file transmission engine */
class ZMODEM
{
public:
    virtual void GetData(uint8_t* buffer, uint32_t position, unsigned length) = 0;
    virtual void SendByte(uint8_t byte) = 0;
    virtual bool CharAvail() = 0;
    virtual bool Carrier() = 0;
    virtual int GetByte(int tenths = 100) = 0;
    // ^ Results: byte, or RCDO for !Carrier, or TIMEOUT

    enum {
        // Message types:
        ZRQINIT = 0,     /* Request receive-init */
        ZRINIT = 1,      /* Receive init */
        ZSINIT = 2,      /* Send init sequence (optional) */
        ZACK = 3,        /* Acknowledge previous message */
        ZFILE = 4,       /* File name from sender */
        ZSKIP = 5,       /* To sender: skip this file */
        ZNAK = 6,        /* Negative acknowledge: Last packet was garbled */
        ZABORT = 7,      /* Abort batch transfers */
        ZFIN = 8,        /* Finish session */
        ZRPOS = 9,       /* Resume data trans at this position */
        ZDATA = 10,      /* Data packet(s) follow */
        ZEOF = 11,       /* End of file */
        ZFERR = 12,      /* Fatal Read or Write error Detected */
        ZCRC = 13,       /* Request for file CRC and response */
        ZCHALLENGE = 14, /* Receiver's Challenge */
        ZCOMPL = 15,     /* Request is complete */
        ZCAN = 16,       /* Other end canned session with CAN*5 */
        ZFREECNT = 17,   /* Request for free bytes on filesystem */
        ZCOMMAND = 18,   /* Command from sending program */
        ZSTDERR = 19,    /* Output to standard error, data follows */
        // Special characters:
        ZPAD  = '*',           /* 052 Padding character begins frames */
        ZDLE  = 030, CAN=ZDLE, /* Ctrl-X ZMODEM escape - `ala BISYNC DLE */
        ZBIN  = 'A',           /* Binary frame indicator */
        ZHEX  = 'B',           /* HEX frame indicator */
        ZBIN32 = 'C',
        XON  = 021, CR = 015, DLE = 020,
        XOFF = 023, LF = 012,
        // ZDLE sequences:
        ZCRCE = 0x68,     /* CRC next, frame ends, header packet follows */
        ZCRCG = 0x69,     /* CRC next, frame continues nonstop */
        ZCRCQ = 0x6A,     /* CRC next, frame continues, ZACK expected */
        ZCRCW = 0x6B,     /* CRC next, ZACK expected, end of frame */
        ZRUB0 = 0x6C,     /* Translate to rubout 0177 */
        ZRUB1 = 0x6D,     /* Translate to rubout 0377 */
        // Generic error codes
        ERROR   = -1,
        TIMEOUT = -2,
        RCDO    = -3,
    };

public:
    /*--------------------------------------------------------------------*/
    /* Send a file: Returns true for successful transfer, false for error */
    /*--------------------------------------------------------------------*/
    bool Send(const char* filename, uint32_t file_length);

private:
    /* Macro for sending multiple bytes at once (this compiles only in C++11) */
    template<typename... T>
    void SendByte(uint8_t b, T... rest)
    {
        SendByte(b);
        SendByte(rest...);
    }
};



