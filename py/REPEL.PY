import robot

CW=0
CCW=1
FORWARD=0
REVERSE=1

turn90 = 86
movepwm = 100

class RepelAction:
    def __init__(self,dist):
        self.dist = dist
        self.alpha = 10

    def _start(self):
        print("REPEL START")

    def _loop(self):
        lp = robot.proximity(1)
        rp = robot.proximity(2)

        ldiff = lp - self.dist
        rdiff = rp - self.dist

        if ((lp > 40 and rp > 40) or (lp < 5 and rp < 5)):
            robot.stop()
        elif (abs(ldiff) <= 1 and abs(rdiff) <= 1):
            robot.stop()
        elif (lp > (rp + 3) and (rp < 15)):
            # turn to the right
            robot.spin_pwm(CW,100)
        elif (rp > (lp + 3) and (lp < 15)):
            # turn to the left
            robot.spin_pwm(CCW,100)
        else:
            p = (lp + rp)/2
            dir = FORWARD
            diff = p - self.dist
            if (diff < 0):
                dir = REVERSE
                diff = -2 * diff
            if (diff > self.alpha): diff = self.alpha
            speed = 255 * diff / self.alpha
            robot.move_pwm(dir,speed)
        return 1

    def _end(self):
        print("REPEL END")

    def _dump(self):
        print("Python repel")

repel = RepelAction(12)

def startTestAction():
	robot.action(repel)

robot.doTest = startTestAction
