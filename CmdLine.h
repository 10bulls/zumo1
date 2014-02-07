#ifndef CmdLine_h_defined
#define CmdLine_h_defined

#define CMD_BUFFER_LEN 256

class CmdLine
{
public:
	CmdLine(Stream * s = &Serial, const char * prompt=0)
	{
		input = s;
		output = s;
		ibuff = 0;
		buff[0] = 0;
		CommandReady = false;
		_prompt = prompt;
		_escape = 0;
	}

	virtual char * Command()
	{
		return buff;
	}

	virtual boolean CheckStream( Stream * s)
	{
		if (s->available() == 0) return false;

		CommandReady = false;

		int ch = s->read();

		// ANSI escape codes
		// http://en.wikipedia.org/wiki/ANSI_escape_code
		if (_escape == 1)
		{
			if (ch == '[')
				_escape++;
			else
				// unrecognised ESC sequence...
				_escape = 0;
			return true;
		}

		if (_escape == 2)
		{
			switch(ch)
			{
			case 'A':
				CursorUp();
				break;
			case 'B':
				CursorDown();
				break;
			case 'C':
				CursorRight();
				break;
			case 'D':
				CursorLeft();
				break;
			}
			_escape = 0;
			return true;
		}
		
		// use this stream for output
		// robot.sout = s;
		// _xmodem_s = s;
		if (input != s)
		{
			input = s;
			output = s;
		}

		if (ch == 8)
		{
			// backspace
			if (ibuff)
			{
				s->print("\b \b");
				ibuff--;
			}
		}
		else if (ch == 13)
		{
			NewLine();
			buff[ibuff] = 0;
			ibuff = 0;
			//parse_serial_buffer(serial_buffer);
			CommandReady = true;
		}
		else if (ch == 27)
		{
			// escape
			_escape = 1;
		}
		else if (ch >= 32 && ch <= 126)	// ignore control characters
		{
			// echo the character
			s->write(ch);
			if (ibuff > sizeof(buff)-1)
			{
				ibuff = 0;
			}
			buff[ibuff] = ch;
			ibuff++;
		}
		return true;
	}

	virtual void CursorUp()
	{
		// poor man's command history
		for(int i=ibuff; i > 0; i--)
			output->write("\b");
		output->write(buff);
		for(ibuff=0;ibuff < sizeof(buff)-1 && buff[ibuff]; ibuff++) {}
	}

	virtual void CursorDown()
	{
	}
	
	virtual void CursorLeft()
	{
		if (ibuff) ibuff--;
		output->write("\b");
	}

	virtual void CursorRight()
	{
	}

	virtual void NewLine()
	{
		output->write("\r\n");
	}
	
	virtual void Prompt()
	{
		// *should* already be on newline
		// NewLine();
		if (_prompt) output->write(_prompt);
	}

	Stream * input;
	Stream * output;
	const char * _prompt = 0;
	boolean CommandReady;

private:
	int _escape;
	char buff[CMD_BUFFER_LEN];
	int ibuff;
};


#endif