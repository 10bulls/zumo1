CW=0
CCW=1
FORWARD=0
REVERSE=1

turn90 = 86
movepwm = 100

class myaction:
	def __init__(self):
		self.x = 0

	def _start(self):
		print("PY START")
		self.x = 0

	def _loop(self):
		self.x += 1
		if (self.x == 1000):
			print("LOOP")
			self.x = 0
		return 1

	def _end(self):
		print("PY END")

	def _dump(self):
		print("Test python action")

action1 = myaction()

def startTestAction():
	robot.action(action1)

robot.doTest = startTestAction
