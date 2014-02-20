import robot
import ACTION
from ACTION import *

CW=0
CCW=1
FORWARD=0
REVERSE=1

#
# Stay In Boundary
#
# Use IR reflectance sensor to stay within a boundary.
#
class StayInBoundary(RobotAction):
    def __init__(self,duration):
        RobotAction.__init__(self,duration)
        self.duration_reverse = 150
        self.duration_turn = 150
        # self.speed_fwd = 255
        # self.speed_turn = 255
        self.speed_fwd = 100
        self.speed_turn = 100
        # modes
        # 0 = forward
        # 1 = reversing
        # 2 - turning
        self.mode = 0
        # 0 = not detected
        # -1 = detected to left
        # 1 = detected to right
        # 2 = detected straight
        self.detect = 0
        self.tstart = 0
        self.turn_dir = CW
        self.last_turn_dir = CW
        self.last_turn_time = 0

    def _start(self):
        RobotAction._start(self)
        self.mode = 0
        self.detect = 0
        self.tstart = 0
        self.turn_dir = CW
        self.last_turn_dir = CW
        self.last_turn_time = 0
        robot.move_pwm(FORWARD,self.speed_fwd)

    def _loop(self):
        if (RobotAction._loop(self)==0):
            return 0

        tnow = pyb.millis()
        if (self.mode == 1):
            # reversing
            if (tnow - self.tstart >= self.duration_reverse + pyb.random(100)):
                # turn
                robot.spin_pwm(self.turn_dir,self.speed_turn)
                self.mode = 2
                self.tstart = tnow
            return 1
        
        if (self.mode == 2):
            # turning
            if (tnow - self.tstart >= self.duration_reverse + pyb.random(100)):
                # forward
                robot.move_pwm(FORWARD,self.speed_fwd)
                self.mode = 0
                self.tstart = tnow

        # read raw sensor values
        robot.reflect_read()
        self.detect = 0
        for i in range(0,robot.reflect_num_sensors()):
            if (robot.reflect(i) > 600):
                if (i < robot.reflect_num_sensors()/2):
                    self.detect = -1
                else:
                    self.detect = 1
                break

        if (self.detect):
            self.tstart = pyb.millis()
            # reverse
            if (self.detect < 0):
                # to right
                self.turn_dir = CW
            else:
                # to left
                self.turn_dir = CCW
        
            if ((self.turn_dir != self.last_turn_dir) and (self.tstart-self.last_turn_time < 2 * (self.duration_reverse+self.duration_turn))):
                # if turning CW / CCW in short duration probably stuck in a corner
                self.turn_dir = not self.turn_dir

            if (pyb.random(100) < 20):
                # randomly change direction
                self.turn_dir = not self.turn_dir

            self.last_turn_time = self.tstart
            self.last_turn_dir = self.turn_dir
      
            self.mode = 1

            robot.move_pwm(REVERSE,self.speed_fwd)
        
        # keep going
        return 1

    def _end(self):
        RobotAction._end(self)
        print("boundary end")

    def _dump(self):
        print("Python boundary")

boundary = StayInBoundary(0)

def startBoundary():
	robot.action(boundary)

robot.doTest = startBoundary
