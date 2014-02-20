import robot
import ACTION
from ACTION import *

CW=0
CCW=1
FORWARD=0
REVERSE=1

#
# Explore Action
#
# . Start off with a scan to find clearest path
# . If proximity > threshold then move forward
# . If proximity < threshold then scan again
#
class ExploreAction(RobotAction):
    def __init__(self,duration):
        RobotAction.__init__(self,duration)
        self.scan = ScanAction(CW,180,150,20,3000)
        self.move = MoveAction(CW,150,15,5000)
        # mode 
        #   0 = scan
        #   1 = drive
        self.mode = 0

    def _start(self):
        RobotAction._start(self)
        self.mode = 0
        self.scan._start()

    def _loop(self):
        if (RobotAction._loop(self)==0):
            return 0
        if (self.mode == 0):
            # scan mode
            if (self.scan._loop()==0):
                # scan finished, go to move mode
                self.mode = 1
                self.scan._end()
                self.move._start()
        else:
            # move mode
            if (self.move._loop()==0):
                # move finished, go to scan
                self.mode = 0
                self.move._end()
                self.scan._start()
        # keep going
        return 1

    def _end(self):
        RobotAction._end(self)
        self.scan._end()
        self.move._end()
        print("Explore end")

    def _dump(self):
        print("Python explore")

explore = ExploreAction(0)

def startExploreAction():
	robot.action(explore)

robot.doTest = startExploreAction
