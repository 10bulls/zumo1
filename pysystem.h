// pysystem.h

#ifndef _PYSYSTEM_h
#define _PYSYSTEM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void * mp_obj_t;

void python_setup(void);
int xpy_main(void);
void run_python_cmd_str( const char * cmd );
bool do_file(const char *filename);
void python_test_call();
bool python_robot_event(const char * handler);

mp_obj_t find_python_object( const char * name );
mp_obj_t find_python_attribute( mp_obj_t o, const char * attrib_name );
mp_obj_t find_python_method( mp_obj_t o, const char * method_name );
int python_call_method( mp_obj_t m, mp_obj_t fun );

#ifdef __cplusplus
}
#endif

#endif

