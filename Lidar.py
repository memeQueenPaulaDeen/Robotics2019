from rplidar import RPLidar, RPLidarException
import time
import threading
import sys
import os
import signal
import RPi.GPIO as GPIO
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
    numScans = 24


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
        desiredAngles = [x*360/self.numScans for x in range(self.numScans)]

        for scan in dataScan:
            angles = [reading[self.angleIdx] for reading in scan]
            distances = [reading[self.distanceIdx] for reading in scan]

            for angle in desiredAngles:

                min_index = angles.index(min(angles,key=lambda x:180-abs(abs(x-angle) - 180)))

                measures.update({angle: [15, angles[min_index], distances[min_index]]})



        #print(measures)
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
        for x in range(self.numScans):
            print(measures[x*360/self.numScans])

    def convertMeasures(self,measures):
        m = []
        num = self.numScans
        for x in range(num):
            m.append([measures[(x*-360/num)%360][self.distanceIdx],(x*360/num)%360])#reverse scan order (- 45 first)
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