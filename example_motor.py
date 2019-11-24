import time
from adafruit_motorkit import MotorKit
from threading import Thread


class Motor():


    kit = MotorKit()

    def setLeft(self,throt):
        self.kit.motor1.throttle= throt

    def setRight(self,throt):
        self.kit.motor4.throttle = throt

    def setPhiDotDesiredRight(self,phiDot):
        throt = 0
        if phiDot < 0:
            throt = -.3+(.7*phiDot/3.8)
            throt = max(-1,throt)
        if phiDot > 0:
            throt = .3+(.7*phiDot/3.8)
            throt = min(1,throt)
        #print("Throtle right " + str(throt))
        self.setRight(throt)

    def setPhiDotDesiredLeft(self,phiDot):
        throt = 0
        if phiDot < 0:
            throt = -.3+(.7*phiDot/3.8)
            throt = max(-1,throt)
        if phiDot > 0:
            throt = .3+(.7*phiDot/3.8)
            throt = min(1,throt)
        #print("Throtle left " + str(throt))
        self.setLeft(throt)



    def brake(self):
        self.kit.motor1.throttle = 0
        self.kit.motor4.throttle = 0
        time.sleep(1)

    def off(self):
        self.kit.motor1.throttle = None
        self.kit.motor4.throttle = None

#motor1 is left motor; motor4 is right motor
# throttle must go from -1 to 1
# 0 brakes, None is free to rotate 
# to run file:
# sudo python3 example_motor.py

#go forward
# kit.motor1.throttle = 0.8
# kit.motor4.throttle = 0.8
# time.sleep(5)
#
# #go backward
# kit.motor1.throttle = -0.8
# kit.motor4.throttle = -0.8
# time.sleep(2)
#
# #brake
# kit.motor1.throttle = 0
# kit.motor4.throttle = 0
# time.sleep(1)
#
# #release motors
# kit.motor1.throttle = None
# kit.motor4.throttle = None


