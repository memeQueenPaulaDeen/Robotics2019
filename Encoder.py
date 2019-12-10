
import time
import RPi.GPIO as GPIO
import threading
from threading import Thread
import math
import numpy as np

class Encoder(Thread):
	GPIO.setmode(GPIO.BCM)

	#leftDistance = 0
	#rightDistance = 0


	leftTime = time.time()
	rightTime = time.time()

	timeStep = .2
	rightCount = 0
	lastRightCount = 0
	lastLeftCount = 0
	leftCount = 0

	ticksInRev = 2240
	wheelRadiusCm = 4.5
	wheelBaseCm = 22

	x_body = 0
	y_body = 0

	# assume Inertial frame origin is start pose
	x_inertial_enc = 0
	y_inertial_enc = 0
	theata_enc = 0

	x_inertial = 0
	y_inertial = 0
	theata = 0

	phiDotLeft = 0
	phiDotRight = 0

	start_x_cm = None
	start_y_cm = None
	start_th = None

	# def __init__(self,):
	# 	Thread.__init__() #todo



	def run(self):
		pin_A_left = 17
		pin_B_left = 18
		pin_A_right = 20
		pin_B_right = 21

		GPIO.setup(pin_A_left, GPIO.IN)
		GPIO.setup(pin_B_left, GPIO.IN)
		GPIO.setup(pin_A_right, GPIO.IN)
		GPIO.setup(pin_B_right, GPIO.IN)

		outcome = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]

		last_AB_left = 0b00
		last_AB_right = 0b00
		counter_left = 0
		counter_right = 0

		thread = threading.Thread(target=self.encoderUpdate,args=((self.timeStep,)))
		thread.start()

		while True:
			A_left = GPIO.input(pin_A_left)
			B_left = GPIO.input(pin_B_left)

			A_right = GPIO.input(pin_A_right)
			B_right = GPIO.input(pin_B_right)

			current_AB_left = (A_left << 1) | B_left
			current_AB_right = (A_right << 1) | B_right



			if (current_AB_left != last_AB_left):
				position_left = (last_AB_left << 2) | current_AB_left
				counter_left += outcome[position_left]
				last_AB_left = current_AB_left
				self.leftCount = -counter_left

				# left motor is backwards
				#print("Left wheel counter: ", -self.counter_left)

			if (current_AB_right != last_AB_right):
				position_right = (last_AB_right << 2) | current_AB_right
				counter_right += outcome[position_right]
				last_AB_right = current_AB_right
				self.rightCount = counter_right
				#print("Right wheel counter: ", self.counter_right)


	def getRightTicks(self):
		diff = self.rightCount - self.lastRightCount
		self.lastRightCount = self.rightCount
		return diff

	def getLeftTicks(self):
		diff = self.leftCount - self.lastLeftCount
		self.lastLeftCount = self.leftCount
		return diff

	def getPhiDotRight(self):
		timeDelta = time.time() - self.rightTime
		self.rightTime = time.time()
		rightDistance = (2*math.pi*self.wheelRadiusCm*self.getRightTicks()/self.ticksInRev)
		return rightDistance, rightDistance/(timeDelta*self.wheelRadiusCm)

	def getPhiDotLeft(self):
		timeDelta = time.time() - self.leftTime
		self.leftTime = time.time()
		leftDistance = (2*math.pi*self.wheelRadiusCm*self.getLeftTicks()/self.ticksInRev)
		return leftDistance , leftDistance/(timeDelta*self.wheelRadiusCm)

	def getDeltas(self):
		leftDistance, phiDotLeft = self.getPhiDotLeft()
		rightDistance, phiDotRight = self.getPhiDotRight()
		totalDistance = (leftDistance+rightDistance)/2
		deltaThetaRad=(rightDistance-leftDistance)/self.wheelBaseCm

		deltaX = 0
		deltaY= 0

		if deltaThetaRad != 0:
			RadiusOfCurve=totalDistance/deltaThetaRad
			deltaX = RadiusOfCurve*math.sin(deltaThetaRad)
			deltaY = RadiusOfCurve*(1-math.cos(deltaThetaRad))
		else:
			deltaX = totalDistance

		return deltaX , deltaY , deltaThetaRad, phiDotLeft, phiDotRight

	def encoderUpdate(self,timeStep):

		while True:
			time.sleep(timeStep)

			deltaX, deltaY, deltaThetaRad, phiDotLeft, phiDotRight = self.getDeltas()
			self.x_body = self.x_body + deltaX
			self.y_body = self.y_body + deltaY
			self.x_inertial_enc = (math.cos(self.theata_enc) * deltaX - math.sin(self.theata_enc) * deltaY) + self.x_inertial_enc
			self.y_inertial_enc = (math.sin(self.theata_enc) * deltaX + math.cos(self.theata_enc) * deltaY) + self.y_inertial_enc
			self.theata_enc = self.theata_enc + deltaThetaRad
			self.phiDotRight = phiDotRight
			self.phiDotLeft = phiDotLeft

			#do transform
			eTb = self.p2t(self.x_inertial_enc, self.y_inertial_enc, self.theata_enc)
			iTe = self.p2t(self.start_x_cm, self.start_y_cm, self.start_th)
			iTb = np.matmul(iTe, eTb)
			newPose = self.t2p(iTb)
			print("new Pose = " + str(newPose))
			self.x_inertial = newPose[0]
			self.y_inertial = newPose[1]
			self.theata = newPose[2]


	def p2t(self, x, y, th):
		return np.array([[np.cos(th), -np.sin(th), x], [np.sin(th), np.cos(th), y], [0, 0, 1]])

	def t2p(self, T):
		return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])

	def Trnasform(self,pos):

		encTinert = self.p2t(pos.start_x_cm,pos.start_y_cm,pos.start_th)
