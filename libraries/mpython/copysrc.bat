@echo off
set src=C:\devt\arduino\micropython

CALL :hcopy %src%\teensy\led.h
CALL :hcopy %src%\teensy\lexermemzip.h
CALL :hcopy %src%\teensy\mpconfigport.h
CALL :hcopy %src%\teensy\usb.h
CALL :hcopy %src%\stm\servo.h
CALL :hcopy %src%\stm\gccollect.h
CALL :hcopy %src%\py\compile.h
CALL :hcopy %src%\py\gc.h
CALL :hcopy %src%\py\lexer.h
CALL :hcopy %src%\py\misc.h
CALL :hcopy %src%\py\mpconfig.h
CALL :hcopy %src%\py\nlr.h
rem CALL :hcopy %src%\py\map.h
rem CALL :hcopy %src%\py\obj.h
CALL :hcopy %src%\py\parse.h
CALL :hcopy %src%\py\parsehelper.h
rem CALL :hcopy %src%\py\qstr.h
CALL :hcopy ..\..\build\py\qstrdefs.generated.h
CALL :hcopy %src%\py\repl.h
CALL :hcopy %src%\py\runtime.h
CALL :hcopy %src%\py\runtime0.h

goto :EOF

:hcopy
echo %1
copy %1 .
goto :EOF   

:end


