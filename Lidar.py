from rplidar import RPLidar, RPLidarException
import time
import threading
import sys
import os
import signal
import RPi.GPIO as GPIO
import example_motor
import numpy as np


PORT_NAME = '/dev/ttyUSB0'
global proxyBool

class Lidar(threading.Thread):
    QualityIdx = 0
    angleIdx = 1
    distanceIdx = 2
    rpLidar = RPLidar(PORT_NAME)
    ml = None
    mr = None
    measures = []



    def run(self):
        global proxyBool
        proxyBool = False
        while not proxyBool:
            time.sleep(.1)
            data = self.readScans(2)
            try:
                m = self.getScan(data)
                #self.prettyPrint(m)
                self.convertMeasures(m)

            except:
                "read error"
                pass



        self.ml.brake()
        self.ml.off()

        print("kill CMD")
        GPIO.cleanup()
        os.killpg(0, signal.SIGTERM)


    def getScan(self,dataScan):
        #print("this is the scan")
        #print(dataScan)
        measures = {}
        desiredAngles = [[(x*45-2)%360,(x*45+2)%360,x*45] for x in range(8)]
        for data in dataScan:
            for reading in data:
                for angles in desiredAngles:
                    if (reading[self.angleIdx] > angles[0] and reading[self.angleIdx] < angles[1]) or \
                        (angles == desiredAngles[0] and (reading[self.angleIdx] > angles[0] or reading[self.angleIdx] < angles[1])):
                        measures.update({angles[2]:reading})

        return measures

                #if reading[self.angleIdx] < 5 or reading[self.angleIdx] >355





    def setForwardProxy(self,dataScan):
        global proxyBool
        dataList = []
        for data in dataScan:  # reading is a full 360 scan
            dataList.append(data)
            for reading in data:

                print(reading)
                if (reading[self.distanceIdx] > 5 and reading[self.distanceIdx]) < 500 and ((reading[self.angleIdx] > 120) or (reading[self.angleIdx] < 260) ): #proxy Trig
                    proxyBool = True






    def readScans(self,scansToCollect):
        #print("this is a test")
        #self.rpLidar.clear_input()
        data = []
        #print("new loop")
        forwardProximity = False
        numOFScans = 0

        try:
            for scan in self.rpLidar.iter_scans():
                # print(scan)
                data.append(correctScan(scan,180))
                numOFScans = numOFScans + 1
                if numOFScans == scansToCollect:
                    return data
            self.rpLidar.stop()
            #fix me
            #self.rpLidar = None
            #self.rpLidar = RPLidar(PORT_NAME)


        except RPLidarException as e:
            #print(e)
            #print("read error")
            self.handleDescriptorErr()
            #self.rpLidar.clear_input()
            pass

    def prettyPrint(self,measures):
        for x in range(8):
            print(measures[x*45])

    def convertMeasures(self,measures):
        m = []
        num = 8
        for x in range(num):
            m.append([measures[(x*-360/num)%360][self.distanceIdx],(x*360/num)%360])
        self.measures = np.array(m)

    def waitForFirstRead(self):
        while len(self.measures) == 0:
            time.sleep(.1)

    def handleDescriptorErr(self):
        # command = [['reverse',.3]]  # ,['forward',.1]]
        # message = bytearray(json.dumps(command), encoding='utf-8')
        # self.clientSocket.sendto(message, self.addr)
        # print("Lidar sent Message: " + str(command))
        #
        # self.serverSocket.recvfrom(12000)
        # print('lidarAck')

        # self.rpLidar.stop()
        # self.rpLidar.stop_motor()
        # self.rpLidar.reset()
        # self.rpLidar.disconnect()
        # self.rpLidar = RPLidar(PORT_NAME)
        self.rpLidar = None
        self.rpLidar = RPLidar(PORT_NAME)

def sortScan(oneScan):
    return sorted(oneScan, key=lambda tup: tup[1])

def correctScan(oneScan,offset):
    return [(data[0],(data[1]+offset)%360,data[2]) for data in oneScan]


def gernerateCheck(anglesToCheck):

    tol = 5
    string = "if (reading[self.angleIdx] <"+  str(5) + "or reading[self.angleIdx] >"+ str(360-5) +") and "
    for x in anglesToCheck:
        string = string + "(reading[self.angleIdx] <"+  str() + "or reading[self.angleIdx] >"+ str(360-5) +")"