import time
from adafruit_motorkit import MotorKit
from threading import Thread
import csv


class Motor():

    phi2MmapRight = {}
    phi2MmapLeft = {}
    debug = False

    def __init__(self):
        with open('rightMotCurve.csv', mode='r') as infileRight:
            reader = csv.reader(infileRight)
            self.phi2MmapRight = {float(rows[1]): float(rows[0]) for rows in reader}
            try:
                self.phi2MmapRight.pop(0.0)
            except:
                print("no 0 key to pop")

        with open('leftMotCurve.csv', mode='r') as infileLeft:
            reader = csv.reader(infileLeft)
            self.phi2MmapLeft = {float(rows[1]): float(rows[0]) for rows in reader}
            try:
                self.phi2MmapLeft.pop(0.0)
            except:
                print("no 0 key to pop")

    kit = MotorKit()

    def setLeft(self,throt):
        if not self.debug:
            self.kit.motor1.throttle= throt
        else:
            self.kit.motor1.throttle = None

    def setRight(self,throt):
        if not self.debug:
            self.kit.motor4.throttle = throt
        else:
            self.kit.motor1.throttle = None

    def setPhiDotDesiredRight(self,phiDot):
        throt = 0
        if phiDot <= 0:
            throt = 0
        if phiDot > 0:
            throt = self.getCMDfromMap(self.phi2MmapRight,phiDot)#.3+(.7*phiDot/3.8)
            throt = min(1,throt)
        #print("Throtle right " + str(throt))
        self.setRight(throt)

    def setPhiDotDesiredLeft(self,phiDot):
        throt = 0
        if phiDot <= 0:
            throt = 0
        if phiDot > 0:
            throt = self.getCMDfromMap(self.phi2MmapLeft,phiDot)#.3+(.7*phiDot/3.8)
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

    def getCMDfromMap(self,maping, phi):
        mydict = maping
        return mydict[phi] if phi in mydict else mydict[min(mydict.keys(), key=lambda k: abs(k - phi))]/100

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


