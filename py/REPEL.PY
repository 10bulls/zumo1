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
		p = robot.proximity()
		if (p > 40 or p < 5):
			robot.stop()
		elif (abs(p - self.dist) <= 1):
			robot.stop()
		else:
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
