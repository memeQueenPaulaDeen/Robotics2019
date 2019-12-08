import signal

import Encoder
import time
import Motor
import PID
import numpy as np
import math
import Lidar
import os
import RPi.GPIO as GPIO
import LocalizationClient as lc
import threading
import Follower
import Position as Pos
import waypoint_follower

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
		while (np.linalg.norm([abs(encoder.x_body), abs(encoder.y_body)]) < dist):
			time.sleep(.2)

			#phiCMD = phiCMD + .1
			PIDleft.control(phiCMD, encoder.phiDotLeft, motors.setPhiDotDesiredLeft)
			PIDRight.control(phiCMD, encoder.phiDotRight, motors.setPhiDotDesiredRight)

			#motors.PID(1, measuredPhiDotLeft, 1.5, motors.setPhiDotDesiredLeft)
			#motors.PID(1, measuredPhiDotRight, 1.5, motors.setPhiDotDesiredRight)
			print("Phi Dot Right is: " + str(encoder.phiDotRight))
			print("Phi Dot Left is: " + str(encoder.phiDotLeft))

		print("x " + str(encoder.x_body))
		print("y " + str(encoder.y_body))
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
			delx = encoder.x_body - lastX
			dely = encoder.y_body - lastY
			delTheta = encoder.theata - lastTheta

			sumx += delx
			sumy += dely
			sumTheta += delTheta

			lastX = encoder.x_body
			lastY = encoder.y_body
			lastTheta = encoder.theata

			time.sleep(3)

		print("sum x is: " + str(sumx))
		print("sum y is: " + str(sumy))
		print("sum theta is: " + str(np.degrees(sumTheta)))

################# main ########

	cell_resolution = 50

	# Define Deltas (Not Constant in Real Life)
	dx = 0 * cell_resolution
	dy = 0 * cell_resolution
	dth = 0

	# Instantiate Particle Filter



	try:

		lidar = Lidar.Lidar()
		lidar.start()
		lidar.waitForFirstRead()

		lo_c = lc.LocalizationClient(lidar)
		#lo_c = None
		encoder = Encoder.Encoder()
		encoder.start()

		motors = Motor.Motor()
		#motors.debug = True

		pos = Pos.Position(encoder,lo_c)
		pos.start_x = 5 * cell_resolution
		pos.start_y = 5 * cell_resolution
		pos.start_th = np.radians(90)
		encoder.theata = pos.start_th

		lo_c.sendStartPos(float(pos.start_x), float(pos.start_y), float(pos.start_th))

		lo_c.startCommsThread(pos)

		# Init PID Controllers
		PIDleft = PID.PID(Kp=0.6, Ki=0.25, Kd=0)
		PIDRight = PID.PID(Kp=0.7, Ki=0.3, Kd=0)

		# Define Line Following Parameters
		phi_dot_set = 6
		chi_inf = np.radians(90)
		k_theta_cmd = 0.02
		k_heading_change = 1
		follower = Follower.Follower(phi_dot_set, encoder, PIDleft, PIDRight, motors, chi_inf, k_theta_cmd,
		                             k_heading_change)



		while abs(encoder.x_inertial + encoder.y_inertial) < 100:#np.radians(1) < abs(encoder.theata - targetTh):
			follower.followLine(waypoint_follower.Line(250,250,np.radians(90)),pos.getPoseLocalizationClient())
			print("x inertial " + str(pos.localizationClient.x))
			print("y inertial " + str(pos.localizationClient.y))
			print("theata: " + str(np.degrees(pos.localizationClient.th)))
			time.sleep(.2)

		print("x inertial " + str(encoder.x_inertial))
		print("y inertial " + str(encoder.y_inertial))
		print("theata: " + str(np.degrees(encoder.theata)))

		#Forward(130,encoder,4)
		#Forward(300, encoder, 4)
		motors.brake()
		time.sleep(1)
		motors.off()
		#chariMot(encoder)
		#motors.off()
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






