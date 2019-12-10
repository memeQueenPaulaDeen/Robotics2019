
import numpy as np
import Encoder
import LocalizationClient

class Position():

	encoder = None
	localizationClient = None
	start_x_cm = None
	start_y_cm = None
	start_th = None


	def __init__(self,start_x_cm,start_y_cm,start_th):
		self.encoder = Encoder.Encoder()
		self.encoder.start()

		self.start_x_cm = start_x_cm
		self.start_y_cm = start_y_cm
		self.start_th = start_th

		self.encoder.start_x_cm = start_x_cm
		self.encoder.start_y_cm = start_y_cm
		self.encoder.start_th = start_th

		self.localizationClient = LocalizationClient.LocalizationClient #idk

		#init start x y th for encoder and Loc Client


	def getPoseEcoder(self): # always inertial frame
		return [self.encoder.x_inertial_enc, self.encoder.y_inertial_enc, self.encoder.theata_enc]

	def getPoseLocalizationClient(self):
		return [self.localizationClient.x_cm,self.localizationClient.y_cm,self.localizationClient.th]

	def getStartPos(self):
		return [self.start_x_cm,self.start_y_cm,self.start_th]