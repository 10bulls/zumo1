import robot
import ACTION
from ACTION import *

CW=0
CCW=1
FORWARD=0
REVERSE=1
MAX_SPEED = 120

#
# Line Follow
#
# Use IR reflectance sensor to follow a line.
#
class LineFollow(RobotAction):
    def __init__(self,duration):
        RobotAction.__init__(self,duration)
        self.lastError = 0

    def _start(self):
        RobotAction._start(self)

    def _loop(self):
        if (RobotAction._loop(self)==0):
            return 0

        position = robot.reflect_read_line()

        # error is how far we are away from the center of the line (position 2500)
        error = position - 2500

        pid = error / 2 + 1 * (error - self.lastError)
        pid = pid / 4

        self.lastError = error

        m1Speed = MAX_SPEED + pid
        m2Speed = MAX_SPEED - pid

        if (m1Speed < 0):
            m1Speed = 0
        if (m2Speed < 0):
            m2Speed = 0
        if (m1Speed > MAX_SPEED):
            m1Speed = MAX_SPEED
        if (m2Speed > MAX_SPEED):
            m2Speed = MAX_SPEED
    
        robot.left_pwm(FORWARD,m1Speed)
        robot.right_pwm(FORWARD,m2Speed)
    
        # keep going
        return 1

    def _end(self):
        RobotAction._end(self)
        print("line follow end")

    def _dump(self):
        print("Python line follow")

linefollow = LineFollow(0)

def startLineFollow():
	robot.action(linefollow)

robot.doTest = startLineFollow
