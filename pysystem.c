	#include <stdint.h>
	#include <stdio.h>
	#include <string.h>
	#include <stdlib.h>

	#include "nlr.h"
	#include "misc.h"
	#include "mpconfig.h"
	#include "mpqstr.h"
	#include "obj.h"
	#include "lexer.h"
	#include "lexermemzip.h"
	#include "parse.h"
	#include "compile.h"
	#include "runtime0.h"
	#include "runtime.h"
	#include "repl.h"
	// teensy
	#include "servo.h"
	#include "usb.h"
	#include "led.h"
	// py
	#include "gc.h"

	#include <WProgram.h>

	extern uint32_t _heap_start;

	// #include "pysystem.h"
	/*
	#include <stdint.h>
	#include <stdio.h>
	#include <string.h>
	#include <stdlib.h>

	#include <mpython.h>
	*/
	//void gc_collect(void) {
	//}



	extern uint32_t _heap_start;

	void flash_error(int n) {
		for (int i = 0; i < n; i++) {
			led_state(PYB_LED_BUILTIN, 1);
			delay(250);
			led_state(PYB_LED_BUILTIN, 0);
			delay(250);
		}
	}

	/*
	// from malloc0.c
	void __assert_func(void) {
		// printf("\nASSERT FAIL!");
		for (;;) {
		}
	}
	*/
	/*

	void *calloc(size_t sz, size_t n) {
		char *ptr = malloc(sz * n);
		for (int i = 0; i < sz * n; i++) {
			ptr[i] = 0;
		}
		return ptr;
	}

	void *malloc(size_t n) {
		return gc_alloc(n);
	}

	void free(void *ptr) {
		gc_free(ptr);
	}

	void *realloc(void *ptr, size_t n) {
		return gc_realloc(ptr, n);
	}
	*/


	static const char *help_text = "Hi!\n";

	/*
	static const char *help_text =
	"Welcome to Micro Python!\n\n"
	"This is a *very* early version of Micro Python and has minimal functionality.\n\n"
	"Specific commands for the board:\n"
	"    pyb.info()             -- print some general information\n"
	"    pyb.gc()               -- run the garbage collector\n"
	"    pyb.delay(<n>)         -- wait for n milliseconds\n"
	"    pyb.Led(<n>)           -- create Led object for LED n (n=0)\n"
	"                              Led methods: on(), off()\n"
	"    pyb.gpio(<pin>)        -- read gpio pin\n"
	"    pyb.gpio(<pin>, <val>) -- set gpio pin\n"
	#if 0
	"    pyb.Servo(<n>) -- create Servo object for servo n (n=1,2,3,4)\n"
	"                      Servo methods: angle(<x>)\n"
	"    pyb.switch()   -- return True/False if switch pressed or not\n"
	"    pyb.accel()    -- get accelerometer values\n"
	"    pyb.rand()     -- get a 16-bit random number\n"
	#endif
	;
	*/

	bool do_file(const char *filename);

  
	mp_obj_t pyb_analog_read(mp_obj_t pin_obj) {
		uint pin = mp_obj_get_int(pin_obj);
		int val = analogRead(pin);
		return MP_OBJ_NEW_SMALL_INT(val);
	}


	mp_obj_t pyb_analog_write(mp_obj_t pin_obj, mp_obj_t val_obj) {
		uint pin = mp_obj_get_int(pin_obj);
		int val = mp_obj_get_int(val_obj);
		analogWrite(pin, val);
		return mp_const_none;
	}

	mp_obj_t pyb_analog_write_resolution(mp_obj_t res_obj) {
		int res = mp_obj_get_int(res_obj);
		analogWriteResolution(res);
		return mp_const_none;
	}

	mp_obj_t pyb_analog_write_frequency(mp_obj_t pin_obj, mp_obj_t freq_obj) {
		uint pin = mp_obj_get_int(pin_obj);
		int freq = mp_obj_get_int(freq_obj);
		analogWriteFrequency(pin, freq);
		return mp_const_none;
	}


	// get some help about available functions
	static mp_obj_t pyb_help(void) {
		printf("%s", help_text);
		return mp_const_none;
	}

	// get lots of info about the board
	static mp_obj_t pyb_info(void) {
		// get and print unique id; 96 bits
		{
			byte *id = (byte*)0x40048058;
			printf("ID=%02x%02x%02x%02x:%02x%02x%02x%02x:%02x%02x%02x%02x\n", id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7], id[8], id[9], id[10], id[11]);
		}

		// get and print clock speeds
		printf("CPU=%u\nBUS=%u\nMEM=%u\n", F_CPU, F_BUS, F_MEM);

		// to print info about memory
		{
			extern void *_sdata;
			extern void *_edata;
			extern void *_sbss;
			extern void *_ebss;
			extern void *_estack;
			extern void *_etext;
			printf("_sdata=%p\n", &_sdata);
			printf("_edata=%p\n", &_edata);
			printf("_sbss=%p\n", &_sbss);
			printf("_ebss=%p\n", &_ebss);
			printf("_estack=%p\n", &_estack);
			printf("_etext=%p\n", &_etext);
			printf("_heap_start=%p\n", &_heap_start);
		}

		// GC info
		{
			gc_info_t info;
			gc_info(&info);
			printf("GC:\n");
			printf("  %lu total\n", info.total);
			printf("  %lu used %lu free\n", info.used, info.free);
			printf("  1=%lu 2=%lu m=%lu\n", info.num_1block, info.num_2block, info.max_block);
		}

	#if 0
		// free space on flash
		{
			DWORD nclst;
			FATFS *fatfs;
			f_getfree("0:", &nclst, &fatfs);
			printf("LFS free: %u bytes\n", (uint)(nclst * fatfs->csize * 512));
		}
	#endif

		return mp_const_none;
	}

	// 1FFF8738
	#define RAM_START (0x1FFF8000) // fixed for chip
	#define HEAP_END  (0x20006000) // tunable
	#define RAM_END   (0x20008000) // fixed for chip

	void gc_helper_get_regs_and_clean_stack(machine_uint_t *regs, machine_uint_t heap_end);

	void gc_collect(void) {

	//	return;
  
		uint32_t start = micros();
		gc_collect_start();
		gc_collect_root((void**)RAM_START, (((uint32_t)&_heap_start) - RAM_START) / 4);
		machine_uint_t regs[10];
		gc_helper_get_regs_and_clean_stack(regs, HEAP_END);
		gc_collect_root((void**)HEAP_END, (RAM_END - HEAP_END) / 4); // will trace regs since they now live in this function on the stack
		gc_collect_end();
		uint32_t ticks = micros() - start; // TODO implement a function that does this properly

		if (1) {
			// print GC info
			gc_info_t info;
			gc_info(&info);
			printf("GC@%lu %luus\n", start, ticks);
			printf(" %lu total\n", info.total);
			printf(" %lu used %lu free\n", info.used, info.free);
			printf(" 1=%lu 2=%lu m=%lu\n", info.num_1block, info.num_2block, info.max_block);
		}
	}

	mp_obj_t pyb_gc(void) {
		gc_collect();
		return mp_const_none;
	}

	mp_obj_t pyb_gpio(int n_args, mp_obj_t *args) {
		//assert(1 <= n_args && n_args <= 2);

		uint pin = mp_obj_get_int(args[0]);
		if (pin > CORE_NUM_DIGITAL) {
			goto pin_error;
		}

		if (n_args == 1) {
			// get pin
			pinMode(pin, INPUT);
			return MP_OBJ_NEW_SMALL_INT(digitalRead(pin));
		}
    
		// set pin
		pinMode(pin, OUTPUT);
		digitalWrite(pin, rt_is_true(args[1]));
		return mp_const_none;

	pin_error:
		// nlr_jump(mp_obj_new_exception_msg_1_arg(MP_QSTR_ValueError, "pin %d does not exist", (void *)(machine_uint_t)pin));
		nlr_jump(mp_obj_new_exception_msg_1_arg(MP_QSTR_ValueError, "pin %d does not exist", (const char *)(machine_uint_t)pin));
	}

	MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_gpio_obj, 1, 2, pyb_gpio);

	#if 0
	mp_obj_t pyb_hid_send_report(mp_obj_t arg) {
		mp_obj_t *items = mp_obj_get_array_fixed_n(arg, 4);
		uint8_t data[4];
		data[0] = mp_obj_get_int(items[0]);
		data[1] = mp_obj_get_int(items[1]);
		data[2] = mp_obj_get_int(items[2]);
		data[3] = mp_obj_get_int(items[3]);
		usb_hid_send_report(data);
		return mp_const_none;
	}
	#endif

	static qstr pyb_config_source_dir = 0;
	static qstr pyb_config_main = 0;

	mp_obj_t pyb_source_dir(mp_obj_t source_dir) {
		pyb_config_source_dir = mp_obj_get_qstr(source_dir);
		printf("source_dir = '%s'\n", qstr_str(pyb_config_source_dir));
		return mp_const_none;
	}

	mp_obj_t pyb_main(mp_obj_t main) {
		pyb_config_main = mp_obj_get_qstr(main);
		printf("main = '%s'\n", qstr_str(pyb_config_main));
		return mp_const_none;
	}

	mp_obj_t pyb_delay(mp_obj_t count) {
		delay(mp_obj_get_int(count));
		return mp_const_none;
	}

	mp_obj_t pyb_led(mp_obj_t state) {
		led_state(PYB_LED_BUILTIN, rt_is_true(state));
		return state;
	}

	mp_obj_t pyb_run(mp_obj_t filename_obj) {
		const char *filename = qstr_str(mp_obj_get_qstr(filename_obj));
		do_file(filename);
		return mp_const_none;
	}

	int mpy_strlen(const char *str) {
		int len = 0;
		for (const char *s = str; *s; s++) {
			len += 1;
		}
		return len;
	}

	char *strdup(const char *str) {
		uint32_t len = mpy_strlen(str);
		char *s2 = m_new(char, len + 1);
		memcpy(s2, str, len);
		s2[len] = 0;
		return s2;
	}

	#define READLINE_HIST_SIZE (8)

	static const char *readline_hist[READLINE_HIST_SIZE] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

	void stdout_tx_str(const char *str) {
	//    usart_tx_str(str);
		usb_vcp_send_str(str);
	}

	int readline(vstr_t *line, const char *prompt) {
		stdout_tx_str(prompt);
		int len = vstr_len(line);
		int escape = 0;
		int hist_num = 0;
		for (;;) {
			char c;
			for (;;) {
				if (usb_vcp_rx_any() != 0) {
					c = usb_vcp_rx_get();
					break;
	#if 0
				} else if (usart_rx_any()) {
					c = usart_rx_char();
					break;
	#endif
				}
				//delay(1);
				//if (storage_needs_flush()) {
				//    storage_flush();
				//}
			}
			if (escape == 0) {
				if (c == 4 && vstr_len(line) == len) {
					return 0;
				} else if (c == '\r') {
					stdout_tx_str("\r\n");
					for (int i = READLINE_HIST_SIZE - 1; i > 0; i--) {
						readline_hist[i] = readline_hist[i - 1];
					}
					readline_hist[0] = strdup(vstr_str(line));
					return 1;
				} else if (c == 27) {
					escape = true;
				} else if (c == 127) {
					if (vstr_len(line) > len) {
						vstr_cut_tail(line, 1);
						stdout_tx_str("\b \b");
					}
				} else if (32 <= c && c <= 126) {
					vstr_add_char(line, c);
					stdout_tx_str(line->buf + line->len - 1);
				}
			} else if (escape == 1) {
				if (c == '[') {
					escape = 2;
				} else {
					escape = 0;
				}
			} else if (escape == 2) {
				escape = 0;
				if (c == 'A') {
					// up arrow
					if (hist_num < READLINE_HIST_SIZE && readline_hist[hist_num] != NULL) {
						// erase line
						for (int i = line->len - len; i > 0; i--) {
							stdout_tx_str("\b \b");
						}
						// set line to history
						line->len = len;
						vstr_add_str(line, readline_hist[hist_num]);
						// draw line
						stdout_tx_str(readline_hist[hist_num]);
						// increase hist num
						hist_num += 1;
					}
				}
			} else {
				escape = 0;
			}
			delay(10);
		}
	}

	bool do_file(const char *filename) {
		mp_lexer_t *lex = mp_lexer_new_from_memzip_file(filename);

		if (lex == NULL) {
			printf("could not open file '%s' for reading\n", filename);
			return false;
		}

		qstr parse_exc_id;
		const char *parse_exc_msg;
		mp_parse_node_t pn = mp_parse(lex, MP_PARSE_FILE_INPUT, &parse_exc_id, &parse_exc_msg);

		if (pn == MP_PARSE_NODE_NULL) {
			// parse error
			mp_lexer_show_error_pythonic_prefix(lex);
			printf("%s: %s\n", qstr_str(parse_exc_id), parse_exc_msg);
			mp_lexer_free(lex);
			return false;
		}

		mp_lexer_free(lex);

		mp_obj_t module_fun = mp_compile(pn, false);
		if (module_fun == mp_const_none) {
			return false;
		}

		nlr_buf_t nlr;
		if (nlr_push(&nlr) == 0) {
			rt_call_function_0(module_fun);
			nlr_pop();
			return true;
		} else {
			// uncaught exception
			mp_obj_print((mp_obj_t)nlr.ret_val, PRINT_REPR);
			printf("\n");
			return false;
		}
	}

	void run_python_cmd_str( const char * cmd )
	{
		// mp_lexer_t *lex = mp_lexer_new_from_memzip_file(filename);
		mp_lexer_t *lex = mp_lexer_new_from_str_len("<stdin>", cmd, mpy_strlen(cmd), 0);

		if (lex == NULL) 
		{
			printf("Error creating parser\n");
			return;
		}

		qstr parse_exc_id;
		const char *parse_exc_msg;
		// mp_parse_node_t pn = mp_parse(lex, MP_PARSE_FILE_INPUT, &parse_exc_id, &parse_exc_msg);
		mp_parse_node_t pn = mp_parse(lex, MP_PARSE_SINGLE_INPUT, &parse_exc_id, &parse_exc_msg);

		if (pn == MP_PARSE_NODE_NULL) 
		{
			// parse error
			mp_lexer_show_error_pythonic_prefix(lex);
			printf("%s: %s\n", qstr_str(parse_exc_id), parse_exc_msg);
			mp_lexer_free(lex);
			return;
		}

		mp_lexer_free(lex);

		mp_obj_t module_fun = mp_compile(pn, false);
		if (module_fun == mp_const_none) 
		{
			return;
		}

		nlr_buf_t nlr;
		if (nlr_push(&nlr) == 0) 
		{
			rt_call_function_0(module_fun);
			nlr_pop();
			return;
		}
		else 
		{
			// uncaught exception
			mp_obj_print((mp_obj_t)nlr.ret_val, PRINT_REPR);
			printf("\n");
			return;
		}
	}

	void do_repl(void) {
		stdout_tx_str("Micro Python for Teensy 3.1\r\n");
		stdout_tx_str("Type \"help()\" for more information.\r\n");

		vstr_t line;
		vstr_init(&line, 32);

		for (;;) {
			vstr_reset(&line);
			int ret = readline(&line, ">>> ");
			if (ret == 0) {
				// EOF
				break;
			}

			if (vstr_len(&line) == 0) {
				continue;
			}

			if (mp_repl_is_compound_stmt(vstr_str(&line))) {
				for (;;) {
					vstr_add_char(&line, '\n');
					int len = vstr_len(&line);
					int ret = readline(&line, "... ");
					if (ret == 0 || vstr_len(&line) == len) {
						// done entering compound statement
						break;
					}
				}
			}

			mp_lexer_t *lex = mp_lexer_new_from_str_len("<stdin>", vstr_str(&line), vstr_len(&line), 0);
			qstr parse_exc_id;
			const char *parse_exc_msg;
			mp_parse_node_t pn = mp_parse(lex, MP_PARSE_SINGLE_INPUT, &parse_exc_id, &parse_exc_msg);

			if (pn == MP_PARSE_NODE_NULL) {
				// parse error
				mp_lexer_show_error_pythonic_prefix(lex);
				printf("%s: %s\n", qstr_str(parse_exc_id), parse_exc_msg);
				mp_lexer_free(lex);
			} else {
				// parse okay
				mp_lexer_free(lex);
				mp_obj_t module_fun = mp_compile(pn, true);
				if (module_fun != mp_const_none) {
					nlr_buf_t nlr;
					uint32_t start = micros();
					if (nlr_push(&nlr) == 0) {
						rt_call_function_0(module_fun);
						nlr_pop();
						// optional timing
						if (0) {
							uint32_t ticks = micros() - start; // TODO implement a function that does this properly
							printf("(took %lu ms)\n", ticks);
						}
					} else {
						// uncaught exception
						mp_obj_print((mp_obj_t)nlr.ret_val, PRINT_REPR);
						printf("\n");
					}
				}
			}
		}

		stdout_tx_str("\r\n");
	}

	void python_setup(void)
	{
		// GC init
		gc_init(&_heap_start, (void*)HEAP_END);

		qstr_init();
		rt_init();

		// add some functions to the python namespace
		{
			rt_store_name(qstr_from_str_static("help"), rt_make_function_n(0, (void*)pyb_help));
			mp_obj_t m = mp_obj_new_module(qstr_from_str_static("pyb"));
			rt_store_attr(m, qstr_from_str_static("info"), rt_make_function_n(0, (void*)pyb_info));
			rt_store_attr(m, qstr_from_str_static("source_dir"), rt_make_function_n(1, (void*)pyb_source_dir));
			rt_store_attr(m, qstr_from_str_static("main"), rt_make_function_n(1, (void*)pyb_main));
			rt_store_attr(m, qstr_from_str_static("gc"), rt_make_function_n(0, (void*)pyb_gc));
			rt_store_attr(m, qstr_from_str_static("delay"), rt_make_function_n(1, (void*)pyb_delay));
			rt_store_attr(m, qstr_from_str_static("led"), rt_make_function_n(1, (void*)pyb_led));
			rt_store_attr(m, qstr_from_str_static("Led"), rt_make_function_n(1, (void*)pyb_Led));
			rt_store_attr(m, qstr_from_str_static("analogRead"), rt_make_function_n(1, (void*)pyb_analog_read));
			rt_store_attr(m, qstr_from_str_static("analogWrite"), rt_make_function_n(2, (void*)pyb_analog_write));
			rt_store_attr(m, qstr_from_str_static("analogWriteResolution"), rt_make_function_n(1, (void*)pyb_analog_write_resolution));
			rt_store_attr(m, qstr_from_str_static("analogWriteFrequency"), rt_make_function_n(2, (void*)pyb_analog_write_frequency));

			rt_store_attr(m, qstr_from_str_static("gpio"), (mp_obj_t)&pyb_gpio_obj);
			rt_store_attr(m, qstr_from_str_static("Servo"), rt_make_function_n(0, (void*)pyb_Servo));
			rt_store_name(qstr_from_str_static("pyb"), m);
			rt_store_name(qstr_from_str_static("run"), rt_make_function_n(1, (void*)pyb_run));
		}
	}

	int xpy_main(void) {

		pinMode(LED_BUILTIN, OUTPUT);

		/*
		for(;;)
		{
			digitalWrite(LED_BUILTIN,1);
			delay(1000);
			digitalWrite(LED_BUILTIN,0);
			delay(1000);
		}
		*/

	#if 0
		// Wait for host side to get connected
		while (!usb_vcp_is_connected()) {
			;
		}
	#else
		delay(1000);
	#endif

		led_init();
		led_state(PYB_LED_BUILTIN, 1);

	//    int first_soft_reset = true;

	soft_reset:

		// GC init
		gc_init(&_heap_start, (void*)HEAP_END);

		qstr_init();
		rt_init();

		// add some functions to the python namespace
		{
			rt_store_name(qstr_from_str_static("help"), rt_make_function_n(0, (void*)pyb_help));
			mp_obj_t m = mp_obj_new_module(qstr_from_str_static("pyb"));
			rt_store_attr(m, qstr_from_str_static("info"), rt_make_function_n(0, (void*)pyb_info));
			rt_store_attr(m, qstr_from_str_static("source_dir"), rt_make_function_n(1, (void*)pyb_source_dir));
			rt_store_attr(m, qstr_from_str_static("main"), rt_make_function_n(1, (void*)pyb_main));
			rt_store_attr(m, qstr_from_str_static("gc"), rt_make_function_n(0, (void*)pyb_gc));
			rt_store_attr(m, qstr_from_str_static("delay"), rt_make_function_n(1, (void*)pyb_delay));
			rt_store_attr(m, qstr_from_str_static("led"), rt_make_function_n(1, (void*)pyb_led));
			rt_store_attr(m, qstr_from_str_static("Led"), rt_make_function_n(1, (void*)pyb_Led));
			rt_store_attr(m, qstr_from_str_static("analogRead"), rt_make_function_n(1, (void*)pyb_analog_read));
			rt_store_attr(m, qstr_from_str_static("analogWrite"), rt_make_function_n(2, (void*)pyb_analog_write));
			rt_store_attr(m, qstr_from_str_static("analogWriteResolution"), rt_make_function_n(1, (void*)pyb_analog_write_resolution));
			rt_store_attr(m, qstr_from_str_static("analogWriteFrequency"), rt_make_function_n(2, (void*)pyb_analog_write_frequency));

			rt_store_attr(m, qstr_from_str_static("gpio"), (mp_obj_t)&pyb_gpio_obj);
			rt_store_attr(m, qstr_from_str_static("Servo"), rt_make_function_n(0, (void*)pyb_Servo));
			rt_store_name(qstr_from_str_static("pyb"), m);
			rt_store_name(qstr_from_str_static("run"), rt_make_function_n(1, (void*)pyb_run));
		}

		printf("About execute /boot.py\n");
		if (!do_file("/boot.py")) {
			printf("Unable to open '/boot.py'\n");
			flash_error(4);
		}
		printf("Done executing /boot.py\n");

		// Turn bootup LED off
		led_state(PYB_LED_BUILTIN, 0);

		// run main script
		{
			vstr_t *vstr = vstr_new();
			vstr_add_str(vstr, "/");
			if (pyb_config_source_dir == 0) {
				vstr_add_str(vstr, "src");
			} else {
				vstr_add_str(vstr, qstr_str(pyb_config_source_dir));
			}
			vstr_add_char(vstr, '/');
			if (pyb_config_main == 0) {
				vstr_add_str(vstr, "main.py");
			} else {
				vstr_add_str(vstr, qstr_str(pyb_config_main));
			}
			printf("About execute '%s'\n", vstr_str(vstr));
			if (!do_file(vstr_str(vstr))) {
				printf("Unable to open '%s'\n", vstr_str(vstr));
				flash_error(3);
			}
			printf("Done executing '%s'\n", vstr_str(vstr));
			vstr_free(vstr);
		}

		do_repl();

		printf("PYB: soft reboot\n");

	//    first_soft_reset = false;
		goto soft_reset;
		return 0;
	}

	double sqrt(double x) {
		// TODO
		return 0.0;
	}

	machine_float_t machine_sqrt(machine_float_t x) {
		// TODO
		return x;
	}

#ifdef HAVE_INITFINI_ARRAY

/* These magic symbols are provided by the linker.  */
extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));
extern void (*__fini_array_start []) (void) __attribute__((weak));
extern void (*__fini_array_end []) (void) __attribute__((weak));

extern void _init (void);
extern void _fini (void);

// Need these if using nostdlib
// Don't need these if using nodefaultlib
#if 0
void _init (void)
{
}

void _fini (void)
{
}
#endif

/* Iterate over all the init routines.  */
void
__libc_init_array (void)
{
  size_t count;
  size_t i;

  count = __preinit_array_end - __preinit_array_start;
  for (i = 0; i < count; i++)
    __preinit_array_start[i] ();

  _init ();

  count = __init_array_end - __init_array_start;
  for (i = 0; i < count; i++)
    __init_array_start[i] ();
}

/* Run all the cleanup routines.  */
void
__libc_fini_array (void)
{
  size_t count;
  size_t i;
  
  count = __fini_array_end - __fini_array_start;
  for (i = count; i > 0; i--)
    __fini_array_start[i-1] ();

  _fini ();
}
#endif

/* Register a function to be called by exit or when a shared library
   is unloaded.  This routine is like __cxa_atexit, but uses the
   calling sequence required by the ARM EABI.  */
int __aeabi_atexit (void *arg, void (*func) (void *), void *d)
{
	// AP: erm... no shared libraries and program doesn't exit
  // return __cxa_atexit (func, arg, d);
}


/*
	char * ultoa(unsigned long val, char *buf, int radix) 	
	{
		unsigned digit;
		int i=0, j;
		char t;

		while (1) {
			digit = val % radix;
			buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
			val /= radix;
			if (val == 0) break;
			i++;
		}
		buf[i + 1] = 0;
		for (j=0; j < i; j++, i--) {
			t = buf[j];
			buf[j] = buf[i];
			buf[i] = t;
		}
		return buf;
	}
	*/

int __errno = 0;