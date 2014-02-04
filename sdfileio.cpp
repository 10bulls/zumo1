//#include <mpython.h>
#include <SD.h>

File _cwd;		// current working directory

void sd_dir(File dir)
{
	if (!dir) return;

	printf("dir %s\n", dir.name());

	dir.rewindDirectory();

	for(;;)
	{
		File f = dir.openNextFile();
		if (!f) break;
		if (f.isDirectory())
			printf("<DIR>");
		else
			printf("     ");

		printf("  %s\r\n",f.name());
		f.close();
	}
}

extern "C" {

void stdout_print_strn_serial(void *data, const char *str, unsigned int len) 
{
	Serial.write(str,len);
}
void stdout_print_strn_serial3(void *data, const char *str, unsigned int len) 
{
	Serial3.write((const uint8_t *)str,len);
}

void sd_dir(const char * path)
{
	if (path)
	{
		File root = SD.open(path);
		if (!root)
		{
			printf("%s not found!\n", path);
			return;
		}
		if (!root.isDirectory())
		{
			printf("%s not a directory!\n", path);
		}
		else
		{
			sd_dir(root);
		}
		root.close();
		return;
	}
	
	if (!_cwd)
	{
		_cwd = SD.open("/");
		if (!_cwd)
		{
			printf("Error reading SD\n");
			return;
		}
	}
	sd_dir(_cwd);
}

struct mymp_lexer_file_buf
{
	File * fp;
	char buf[20];
	uint16_t len;
	uint16_t pos;
};

void cpp_file_buf_close(void *fb) 
{
	((mymp_lexer_file_buf*)fb)->fp->close();
	free(fb);
	//f_close(&fb->fp);
	//m_del_obj(mp_lexer_file_buf_t, fb);
}

int cpp_file_buf_next_char(void *vfb) 
{
	mymp_lexer_file_buf* fb = (mymp_lexer_file_buf*)vfb;

	if (fb->pos >= fb->len) 
	{
		if (fb->len < sizeof(fb->buf)) 
		{
			// return MP_LEXER_CHAR_EOF;
			return -1;
		} 
		else 
		{
			size_t n = fb->fp->readBytes(fb->buf, sizeof(fb->buf));
			if (n == 0) 
			{
				// return MP_LEXER_CHAR_EOF;
				return -1;
			}
			fb->len = n;
			fb->pos = 0;
		}
	}
	return fb->buf[fb->pos++];
}

File _sd_f;

void * cpp_lexer_new_from_file(const char *filename) 
{
	_sd_f = SD.open(filename);

	if (!_sd_f) return 0;

	mymp_lexer_file_buf * fb = (mymp_lexer_file_buf*)malloc(sizeof(mymp_lexer_file_buf));

	fb->fp = &_sd_f;

	size_t n = fb->fp->readBytes(fb->buf, sizeof(fb->buf));
	fb->len = n;
	fb->pos = 0;
	return (void*)fb;
}

} // extern "C" 