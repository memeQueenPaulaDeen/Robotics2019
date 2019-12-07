
import numpy as np

class Position():

	encoder = None
	localizationClient = None
	start_x = 0
	start_y = 0
	start_th = 0

	def __init__(self,ecoder,localizationClient):
		self.encoder = ecoder
		self.localizationClient = localizationClient


	def getPoseEcoder(self): # always inertial frame
		return [self.encoder.x_inertial,self.encoder.y_inertial,self.encoder.theata]

	def getPoseLocalizationClient(self):
		return [self.localizationClient.x,self.localizationClient.y,self.localizationClient.th]