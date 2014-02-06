#include <SD.h>
#include <XModem.h>

#include <Arduino.h>
#include "zmodem.h"

Stream * _xmodem_s;

File _sd_file;
unsigned long _block_num = 0;


/* Provide serial I/O methods for ZMODEM */
class modem: public ZMODEM
{
public:
    // Transmit byte
    virtual void SendByte(uint8_t byte)
    {
        _xmodem_s->write(byte);
    }
    // Test whether byte is incoming or not
    virtual bool CharAvail()
    {
        return _xmodem_s->available() > 0;
    }
    // Test whether serial line is connected or not
    virtual bool Carrier()
    {
//!        return (*_xmodem_s);
		return true;
    }

    // Receive a byte, or give up within a time limit
    virtual int GetByte(int tenths = 100)
    {
        _xmodem_s->setTimeout(tenths * 100);

        char byte = '?';
        int n = _xmodem_s->readBytes(&byte, 1);

//!        if(!(*_xmodem_s)) return ZMODEM::RCDO; // Disconnected?
        if(!n) return ZMODEM::TIMEOUT; // No response?

        return (uint8_t) byte;
    }

	// Read SD file data from given position
    virtual void GetData(uint8_t* buffer, uint32_t position, unsigned length)
    {
//!		Serial.print("GETDATA P=");
//!		Serial.print(position);
//!		Serial.print(" L=");
//!		Serial.print(length);
		_sd_file.seek(position);
		int n = _sd_file.read(buffer,length);
//!		Serial.print(" N=");
//!		Serial.println(n);
    }

};

int recvChar(int msDelay)
{
	int cnt = 0;
	while(cnt < msDelay)
	{
			if(_xmodem_s->available() > 0)
				return _xmodem_s->read();                   
			delay(1);
			cnt++;
	}
	return -1;
}

void sendChar(char sym)
{
	_xmodem_s->write(sym);
}


bool rcv_dataHandler(unsigned long no, char* data, int size)
{
	if (size==0) return true;

	int n = size;
	for(int i=0; i < size; i++)
	{
		if (data[i] == 0x1a) 
		{
			n = i;
			break;
		}
	}
	if (n) 
		_sd_file.write(data,n);
	return true;
}

uint32_t _send_cnt = 0;

bool send_dataHandler(unsigned long no, char* data, int size)
{
//	printf("BLOCK=%u size=%i _send_cnt=%u\n", (int)no, size, _send_cnt );
	if (_send_cnt == 0) return false;
	uint32_t l = min( _send_cnt, size );

	int n = _sd_file.readBytes(data,l);

	for(int i = n; i < size; i++)
		data[i] = 0x1a;	// fill buffer with CTRL+Z

	_send_cnt -= n;

	return true;

	// return (n == size);
}


extern "C"
{

// void xmode_receive( const char * filename )
void xmode_receive( const char * filename )
{
	if (!_xmodem_s) _xmodem_s = &Serial;

	if (SD.exists((char *)filename))
	{
		SD.remove((char *)filename);
	}
	
	_sd_file = SD.open(filename, FILE_WRITE );

	XModem modem(recvChar, sendChar, rcv_dataHandler);

	_block_num = 0;
	if (modem.receive())
	{
		_xmodem_s->println("\nOK");
	}
	else
	{
		_xmodem_s->println("\nFAIL");
	}

	_sd_file.close();
}

void xmodem_send( const char * filename )
{
	if (!_xmodem_s) _xmodem_s = &Serial;

	if (!SD.exists((char *)filename))
	{
		printf("%s not found.\n",filename);
		return;
	}
	
	_sd_file = SD.open(filename, FILE_READ );
	_send_cnt = _sd_file.size();


	modem zm;
	zm.Send(filename, _send_cnt);
/*
	XModem modem(recvChar, sendChar, send_dataHandler);

	_block_num = 0;
	if (modem.transmit())
	{
		_xmodem_s->println("\nOK");
	}
	else
	{
		_xmodem_s->println("\nFAIL");
	}
*/
	_sd_file.close();
}


}