import time
from adafruit_motorkit import MotorKit
from threading import Thread
import csv


class Motor():

    phi2MmapRight = {}
    phi2MmapLeft = {}
    debug = False

    def __init__(self,runMotorCharicterization=False):

        if runMotorCharicterization:
            #do not contruct normal motor object use only when first setting up the rover
            self.phi2MmapRight = None
            self.phi2MmapLeft = None
        else:
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
        if self.debug:
            self.kit.motor1.throttle = None
        else:
            self.kit.motor1.throttle= throt

    def setRight(self,throt):
        if self.debug:
            self.kit.motor1.throttle = None
        else:
            self.kit.motor4.throttle = throt

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


def chariMot(encoder):
    # create motor curve in local directory (must be the same as dir with code)
    fl = open("leftMotCurve.txt", "x")
    fr = open("rightMotCurve.txt", "x")

    motors = Motor(runMotorCharicterization=True)

    for x in range(20, 100):
        motors.setLeft(x / 100)
        motors.setRight(x / 100)
        time.sleep(1)

        fl.write(str(x) + "," + str(encoder.phiDotLeft) + "\n")
        fr.write(str(x) + "," + str(encoder.phiDotRight) + "\n")
        print("left MotCMD, leftPhiDot " + str(x) + " " + str(encoder.phiDotLeft))
        print("right MotCMD, rightPhiDot " + str(x) + " " + str(encoder.phiDotRight))
        print()

    for x in range(100, 20, -1):
        motors.setLeft(x / 100)
        motors.setRight(x / 100)
        time.sleep(1)

        fl.write(str(x) + "," + str(encoder.phiDotLeft) + "\n")
        fr.write(str(x) + "," + str(encoder.phiDotRight) + "\n")
        print("left MotCMD, leftPhiDot " + str(x) + " " + str(encoder.phiDotLeft))
        print("right MotCMD, rightPhiDot " + str(x) + " " + str(encoder.phiDotRight))
        print()

    fl.close()
    fr.close()
