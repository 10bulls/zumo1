#ifndef mpython_h_defined
#define mpython_h_defined

#ifdef __cplusplus
extern "C" {
#define ENUM_SIZE_TYPE : machine_int_t
#else
#define ENUM_SIZE_TYPE
#endif

#define restrict

#include "nlr.h"
#include "misc.h"
#include "mpconfig.h"
// #include "mpqstr.h"
#include "qstr.h"
#include "lexer.h"
#include "lexermemzip.h"
#include "parse.h"
#include "obj.h"
#include "compile.h"
#include "runtime0.h"
#include "runtime.h"
#include "repl.h"
#include "servo.h"
#include "usb.h"
#include "gc.h"
#include "gccollect.h"
#include "led.h"

#ifdef __cplusplus
}
#endif

#endif