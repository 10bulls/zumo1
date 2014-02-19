from pyb import *

CW=0
CCW=1
FORWARD=0
REVERSE=1

print("huh huh, 'Bottom!'")

def do_nothing():
    return

robot.onEnd = do_nothing

print("Loading explore.py...")
run("explore.py")

# Press x or remote play to start explore