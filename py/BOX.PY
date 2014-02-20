# TODO ... turn this into a robot action
import robot

CW=0
CCW=1
FORWARD=0
REVERSE=1

turn90 = 86
movepwm = 100

def mForward():
    global movepwm
    global FORWARD
    robot.move(FORWARD,movepwm,0,500)

def mTurn():
    global turn90
    global movepwm
    global CW
    robot.spin(CW,turn90,movepwm,2000)

def mFinished():
    global nmove
    robot.rest()

nmove = -1

moves = [ 
	mForward, 
	mTurn,
	mForward, 
	mTurn,
	mForward, 
	mTurn,
	mForward,
	mTurn,
	mFinished ]
	
def nextMove():
	global nmove
	global moves
	if (nmove < 0 or nmove >= len(moves)): return 0
	moves[nmove]()
	nmove += 1
	if (nmove >= len(moves)): return 0
	return 1

def doBox():
    robot.onEnd = nextMove
    global nmove
    global moves
    nmove = 0
    nextMove()

robot.doTest = doBox