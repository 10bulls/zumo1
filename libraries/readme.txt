Copy 'mpython' folder to the arduino libaries folder.

Add the contents of 'boards.txt' to the existing 'arduino/hardware/teensy/boards.txt'
(DO NOT REPLACE THE EXISTING ONE!)
This file contains hard coded paths that will need to be modified.

Copy 'mk20dx256py.ld' to 'arduino/hardware/teensy/cores/teensy3'

Needs Teensyduino 1.18 RC # 2 or later for programs > 128K
http://forum.pjrc.com/threads/24796-Teensyduino-1-18-Release-Candidate-2-Available

Python support can then be added to a project by simply selecting 'Teensy 3.1+Python' as the target board.
