import time
from threading import Thread

import pandas as pd
import os

class Logger(Thread):

	df = None
	filePath = None
	dictList = []
	pos = None

	def __init__(self,filePath,pos):
		Thread.__init__(self)
		self.pos = pos
		self.df = pd.DataFrame([])
		directory = os.path.dirname(filePath)
		self.filePath = directory
		if not os.path.exists(directory):
			os.makedirs(directory)

	def run(self):

		while True:
			try:
				row = {"encoder_X_inertial": self.pos.encoder.x,
				       "encoder_Y_inertial": self.pos.encoder.y_inertial,
				       "encoder_theta_inertial": self.pos.encoder.theata,
				       "localization_X_inertial": self.pos.localizationClient.x_cm,
				       "localization_Y_inertial": self.pos.localizationClient.y_cm,
				       "localization_TH_inertial": self.pos.localizationClient.th
				       }
				self.dictList.append(row)
			except KeyboardInterrupt:
				self.writeLog()

			time.sleep(.5)

	def writeLog(self):
		self.df = pd.DataFrame(self.dictList)
		self.df.to_csv(self.filePath+'log.csv')