//#include <mpython.h>
#include <SD.h>

extern "C" {

// mp_lexer_t *my_mp_lexer_new_from_file(qstr mod_name) {

//	mp_lexer_t *my_mp_lexer_new_from_file(const char *filename) 

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
}