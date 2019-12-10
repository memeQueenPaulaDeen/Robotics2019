import pandas as pd
import os

class Logger():

	df = None
	filePath = None

	def __init__(self,filePath):

		self.df = pd.DataFrame([])
		directory = os.path.dirname(filePath)
		self.filePath = directory
		if not os.path.exists(directory):
			os.makedirs(directory)

	def logData(self,colValuePairs):

		for key in colValuePairs:
			self.df[key]

	def writeLog(self):
		self.df.to_csv(self.filePath+'//log.csv')