import pyb
import robot

class Fred:
    def __init__(self,msg):
        self.msg = msg

    def say(self):
        print(self.msg)

# print(millis())
print("TEST.PY LOAD")
print(dir())

# print(millis())
print(pyb.micros())

print(robot.proximity())
