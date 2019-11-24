import signal

import encoder_mytrial
import time
import example_motor
import PID
import numpy as np
import math
import Lidar
import os
import RPi.GPIO as GPIO
import LocalizationClient as lc
import threading

if __name__ == "__main__":

	def move(motors):
		#time.sleep(8)
		motors.setPhiDotDesiredRight(.7)
		motors.setPhiDotDesiredLeft(.9)
		time.sleep(8)
		motors.off()


	def turn(angle):
		encoder.rightDistance = 0
		encoder.leftDistance = 0
		wheelBase = 21
		wheelRadius = 4
		print("rec dist : " + str((math.pi * (angle * (math.pi / 180)) * wheelBase) / (2 * wheelRadius)))
		while (np.average([abs(encoder.leftDistance), abs(encoder.rightDistance)]) <
			   ((math.pi * (angle * (math.pi / 180)) * wheelBase) / (2 * wheelRadius))):
			time.sleep(.05)
			measuredPhiDotLeft = -1 * encoder.getPhiDotLeft()
			measuredPhiDotRight = encoder.getPhiDotRight()

			PIDleft.control(-.8, measuredPhiDotLeft, motors.setPhiDotDesiredLeft)
			PIDRight.control(.8, measuredPhiDotRight, motors.setPhiDotDesiredRight)

			# motors.PID(1,measuredPhiDotLeft,1.5,motors.setPhiDotDesiredLeft)
			# motors.PID(1,measuredPhiDotRight, 1.5, motors.setPhiDotDesiredRight)
			print("Phi Dot Right is: " + str(measuredPhiDotRight))
			print("Phi Dot Left is: " + str(measuredPhiDotLeft))

		print("left Dist " + str(encoder.leftDistance))
		print("right Dist " + str(encoder.rightDistance))


	def Forward(dist,encoder,phiCMD):

		#phiCMD = 6
		while (np.linalg.norm([abs(encoder.x), abs(encoder.y)]) < dist):
			time.sleep(.2)

			#phiCMD = phiCMD + .1
			PIDleft.control(phiCMD, encoder.phiDotLeft, motors.setPhiDotDesiredLeft)
			PIDRight.control(phiCMD, encoder.phiDotRight, motors.setPhiDotDesiredRight)

			#motors.PID(1, measuredPhiDotLeft, 1.5, motors.setPhiDotDesiredLeft)
			#motors.PID(1, measuredPhiDotRight, 1.5, motors.setPhiDotDesiredRight)
			print("Phi Dot Right is: " + str(encoder.phiDotRight))
			print("Phi Dot Left is: " + str(encoder.phiDotLeft))

		print("x " + str(encoder.x))
		print("y " + str(encoder.y))
		print("theta " +str(np.degrees(encoder.theata)))

	def testLocilization():
		lidar = Lidar.Lidar()
		lidar.start()

		while len(lidar.measures) == 0:
			time.sleep(.1)

		print("going into therad")
		t1 = threading.Thread(target=move, args=(motors,))
		t1.start()
		print("exit")

		measures = []
		while True:
			measures = lidar.measures
			measures = np.append([float(dx), float(dy), float(dth)], measures)

			lo_c.sendData(measures)

			time.sleep(.2)

	def chariMot(encoder):

		fl = open("leftMotCurve.txt","x")
		fr = open("rightMotCurve.txt", "x")


		for x in range(20,100):
			motors.setLeft(x/100)
			motors.setRight(x/100)
			time.sleep(1)

			fl.write(str(x)+","+str(encoder.phiDotLeft)+"\n")
			fr.write(str(x) + "," + str(encoder.phiDotRight) + "\n")
			print("left MotCMD, leftPhiDot "+str(x) + " " + str(encoder.phiDotLeft))
			print("right MotCMD, rightPhiDot "+str(x) + " " + str(encoder.phiDotRight))
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

	def testEncoder(encoder,motors):
		lastX = 0
		lastY = 0
		lastTheta = 0

		t1 = threading.Thread(target=move, args=(motors,))
		t1.start()

		sumx = 0
		sumy = 0
		sumTheta = 0

		for x in range(5):
			delx = encoder.x - lastX
			dely = encoder.y - lastY
			delTheta = encoder.theata - lastTheta

			sumx += delx
			sumy += dely
			sumTheta += delTheta

			lastX = encoder.x
			lastY = encoder.y
			lastTheta = encoder.theata

			time.sleep(3)

		print("sum x is: " + str(sumx))
		print("sum y is: " + str(sumy))
		print("sum theta is: " + str(np.degrees(sumTheta)))

	cell_resolution = 50

	x = 5 * cell_resolution
	y = 45 * cell_resolution
	th = -90  # must send over -th matlab is clockwise python is counter

	# Define Deltas (Not Constant in Real Life)
	dx = 0 * cell_resolution
	dy = 0 * cell_resolution
	dth = 0

	# Instantiate Particle Filter


	#lo_c = lc.LocalizationClient()
	#lo_c.sendData(np.array([float(x), float(y), float(th)]))

	try:
		encoder = encoder_mytrial.Encoder()
		encoder.start()

		motors = example_motor.Motor()

		PIDleft = PID.PID()
		PIDleft.Kd = 0
		PIDleft.Ki =  .25
		PIDleft.Kp = .6

		PIDRight = PID.PID()
		PIDRight.Kd = 0
		PIDRight.Ki = 0.3
		PIDRight.Kp = .7

		Forward(130,encoder,4)
		#Forward(300, encoder, 4)
		motors.brake()
		time.sleep(1)
		motors.off()
		#chariMot(encoder)
		motors.off()
		# for x in range(50):
		#     motors.setRight(.5)
		#     motors.setLeft(.7)
		#     time.sleep(.01/10)
		#     motors.off()
		#     time.sleep(.4/10)
		#testEncoder(encoder,motors)






	except KeyboardInterrupt:

		GPIO.cleanup()
		lo_c.close()
		print("Killed")
		#os.killpg(1, signal.SIGTERM)
		#exit(1)






