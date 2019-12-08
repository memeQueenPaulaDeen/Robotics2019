
import numpy as np
import Encoder
import LocalizationClient

class Position():

	encoder = None
	localizationClient = None


	def __init__(self,start_x,start_y,start_th):
		self.encoder = Encoder.Encoder()
		self.encoder.start()
		self.localizationClient = LocalizationClient.LocalizationClient #idk

		#init start x y th for encoder and Loc Client


	def getPoseEcoder(self): # always inertial frame
		return [self.encoder.x_inertial,self.encoder.y_inertial,self.encoder.theata]

	def getPoseLocalizationClient(self):
		return [self.localizationClient.x,self.localizationClient.y,self.localizationClient.th]