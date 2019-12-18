
import time
import Motor
import PID
import numpy as np
import math
import Lidar
import RPi.GPIO as GPIO
import LocalizationClient as lc
import threading
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

	# Instantiate Particle Filter



	try:

		lidar = Lidar.Lidar()
		lidar.start()
		print("waiting for lidar")
		lidar.waitForFirstRead()
		print("done waiting for lidar")

		lo_c = lc.LocalizationClient(lidar)


		motors = Motor.Motor()
		motors.debug = False


		start_x_mm = 1000 #* cell_resolution
		start_x_cm =start_x_mm/10

		start_y_mm = 250 #* cell_resolution
		start_y_cm = start_y_mm/10

		start_th = np.radians(180)

		pos = Pos.Position(start_x_cm,start_y_cm,start_th,lo_c)

		lo_c.sendStartPos(start_x_mm,start_y_mm,start_th)
		time.sleep(2)
		lo_c.startCommsThread(pos)
		#logger = Logger.Logger('.//',pos)

		while lo_c.y_cm == 0 and lo_c.x_cm == 0 and lo_c.th == 0:
			time.sleep(.1)

		# Init PID Controllers
		PIDleft = PID.PID(Kp=0.6, Ki=0.25, Kd=0)
		PIDRight = PID.PID(Kp=0.7, Ki=0.3, Kd=0)

		updateStateThread = threading.Thread(target=pos.updateState)
		updateStateThread.start()

		while pos.X_fused == None:
			time.sleep(.1)


		wpf = waypoint_follower.WaypointFollower(pos,motors,PIDleft,PIDRight,[[23,25],[30,190]])

		wpf.followWaypoints()








	except KeyboardInterrupt:

		GPIO.cleanup()
		motors.off()
		lo_c.close()
		print("Killed")
		#os.killpg(1, signal.SIGTERM)
		#exit(1)






