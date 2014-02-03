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

void python_setup(void);
int xpy_main(void);
void run_python_cmd_str( const char * cmd );
bool do_file(const char *filename);

#ifdef __cplusplus
}
#endif

#endif

