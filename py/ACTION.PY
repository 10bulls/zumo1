import pyb
import robot

CW=0
CCW=1
FORWARD=0
REVERSE=1

#
# Robot Action 
# a base class for other actions.  Just impliments a timed duration action.
#
class RobotAction:
    def __init__(self,duration):
        self.duration = duration
        self.tstart = 0

    def _start(self):
        self.tstart = pyb.millis()

    def _loop(self):
        if (self.duration > 0):
            if (self.elapsed() > self.duration):
                return 0
        return 1

    def elapsed(self):
        return pyb.millis()-self.tstart

    def _end(self):
        return

    def _dump(self):
        print("python action")

#
# Move Action 
# move robot
#
class MoveAction(RobotAction):
    def __init__(self,dir,pwm,distance,duration):
        RobotAction.__init__(self,duration)
        self.direction = dir
        self.speed_pwm = pwm
        self.distance_target = distance

    def _start(self):
        RobotAction._start(self)
        robot.move_pwm(self.direction, self.speed_pwm)

    def _loop(self):
        if (RobotAction._loop(self)==0):
            return 0
        # TODO: make use of L & R proximity
        prox = robot.proximity(0)
        if (prox < self.distance_target):
            return 0
        # keep going
        return 1

    def _end(self):
        RobotAction._end(self)
        # NOTE: does not stop robot!

    def _dump(self):
        print("python move")

#
# Spin Action 
# spin robot until degree change reaches target angle
#
class SpinAction(RobotAction):
    def __init__(self,dir,degree,pwm,duration):
        RobotAction.__init__(self,duration)
        self.direction = dir
        self.degree_target = degree
        self.speed_pwm = pwm
        self.duration = duration
        self.degrees = 0

    def _start(self):
        RobotAction._start(self)
        if (self.degree_target > 0):
            robot.imu_start()
        robot.spin_pwm(self.direction,self.speed_pwm)
    
    def _loop(self):
        if (RobotAction._loop(self)==0):
            return 0
        if (self.degree_target > 0):
            robot.imu_loop()
            self.degrees = robot.imu_degrees()
            diff = 0
            if (self.direction==CW):
                diff = self.degree_target - self.degrees
            else:
                diff = self.degree_target + self.degrees
            # NOTE: allow 1.2 degrees 'stopping' distance
            # TODO: should instead PID adjust pwm depending on error
            if (diff - 1.2 < 0):
                return 0
        # keep going
        return 1

    def _end(self):
        print("spin end")
        RobotAction._end(self)
        if (self.degree_target > 0):
            robot.stop()
            for i in range(0,50):
                robot.imu_loop()
            self.degrees = robot.imu_degrees()
            robot.imu_stop()
            print("gyro change=")
            print(self.degrees)

    def _dump(self):
        print("Python spin")

#
# Scan Action 
# spin robot until degree change reaches target angle or proximity is greater than a distance target
#
class ScanAction(SpinAction):
    def __init__(self,dir,degree,pwm,distance,duration):
        SpinAction.__init__(self,dir,degree,pwm,duration)
        self.distance_target = distance
        self.distance_ok = 0
        self.degree_ok = 0

    def _start(self):
        self.distance_ok = 0
        self.degree_ok = 0
        # TODO: use proximity L,R to weight this perhaps?
        lprox = robot.proximity(1)
        rprox = robot.proximity(2)

        if (lprox < rprox):
            self.direction = CW
        else:
            self.direction = CCW

        # randomise the initial direction
        #if (pyb.random(100) > 50):
            #self.direction = CW
        #else:
            #self.direction = CCW

        SpinAction._start(self)

    def _loop(self):
        if (SpinAction._loop(self) == 0): 
            return 0
        # TODO: make use of L & R proximity
        prox = robot.proximity(0)
        if (prox >= self.distance_target):
            if (self.distance_ok):
                # if been OK for 15? degrees, end action
                if (abs(robot.imu_degrees() - self.degree_ok) >= 15):
                    return 0
            else:
                self.distance_ok = 1
                self.degree_ok = robot.imu_degrees()
        else:
            self.distance_ok = 0
        # keep going
        return 1

    def _end(self):
        SpinAction._end(self)

    def _dump(self):
        print("Python scan")

