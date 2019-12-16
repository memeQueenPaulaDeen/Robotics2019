import time

import numpy as np
import Encoder
import LocalizationClient

class Position():

	encoder = None
	localizationClient = None
	start_x_cm = None
	start_y_cm = None
	start_th = None

	X_fused = None
	Y_fused = None
	TH_fused = None


	def __init__(self,start_x_cm,start_y_cm,start_th,loc):
		self.encoder = Encoder.Encoder(start_x_cm,start_y_cm,start_th)
		self.encoder.start()

		self.start_x_cm = start_x_cm
		self.start_y_cm = start_y_cm
		self.start_th = start_th

		self.localizationClient = loc#idk

		#init start x y th for encoder and Loc Client

	def updateState(self):
		prevLocPose = self.getPoseLocalizationClient()

		while True:
			delta_dist = np.linalg.norm([self.localizationClient.x_cm - self.encoder.x_inertial,
			                             self.localizationClient.y_cm - self.encoder.y_inertial])
			delta_th = abs(self.localizationClient.th - self.encoder.theata)

			r = .1
			if delta_dist > 15 or delta_th > np.radians(30) or self.getPoseLocalizationClient() != prevLocPose:#self.encoder.phiDotSet * self.encoder.wheelRadiusCm * self.localizationClient.delta_t:
				r = 0

			fusedPose = np.dot(r,self.getPoseLocalizationClient()) + np.dot((1-r),self.getPoseEcoder())
			if r == 0:
				print("fusedPose " + str(fusedPose) + " from encoders " +
				      " delta_dist > 25 is " + str(delta_dist > 15) +
				      " self.getPoseLocalizationClient() != prevLocPose " +
				      str(self.getPoseLocalizationClient() != prevLocPose))
			else:
				print("fusedPose " + str(fusedPose) + " from localization")

			self.X_fused = fusedPose[0]
			self.Y_fused = fusedPose[1]
			self.TH_fused = fusedPose[2]

			self.encoder.x_inertial = self.X_fused
			self.encoder.y_inertial = self.Y_fused
			self.encoder.theata = self.TH_fused

			prevLocPose = self.getPoseLocalizationClient()

			time.sleep(.2)

	def getPoseEcoder(self): # always inertial frame
		return [self.encoder.x_inertial, self.encoder.y_inertial, self.encoder.theata]

	def getPoseLocalizationClient(self):
		return [self.localizationClient.x_cm,self.localizationClient.y_cm,self.localizationClient.th]

	def getStartPos(self):
		return [self.start_x_cm,self.start_y_cm,self.start_th]

	def getDeltaTh(self, th_old, th_new):
		if np.sign(th_old) != np.sign(th_new):
			return th_old + th_new
		else:
			return th_old - th_new