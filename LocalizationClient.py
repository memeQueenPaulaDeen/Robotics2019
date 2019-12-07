import socket
import time
import threading
import array
import numpy as np

class LocalizationClient():

	#TCP_IP = '127.0.0.1'
	TCP_IP = '192.168.1.5'
	TCP_PORT = 42069
	BUFFER_SIZE = 1000
	# MESSAGE = "Hello, World!"
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	lidar = None
	x = 0
	y = 0
	th = 0

	def __init__(self,lidar):
		self.s.connect((self.TCP_IP, self.TCP_PORT))
		print("connected to server")
		self.lidar = lidar


	def sendData(self,MESSAGE):
		try:
			sendThread = threading.Thread(target=self.s.sendall,args=(MESSAGE,))
			sendThread.start()
			#self.s.sendall(MESSAGE)
		except BrokenPipeError as e:
			print(e)
			self.close()
			raise RuntimeError()

	def reciveMessage(self):
		data = self.s.recv(self.BUFFER_SIZE)
		data = array.array('d', data)
		return data

	def close(self):
		self.s.close()

	def sendStartPos(self,x,y,th):
		self.sendData(np.array([float(x), float(y), float(th)]))

	def sendPKTtoMatlab(self,pos):

		x = pos.encoder.x_inertial
		y = pos.encoder.y_inertial
		th = pos.encoder.theata


		while True:

			measures = self.lidar.measures
			measures = np.append([float(x-pos.encoder.x_inertial), float(y-pos.encoder.y_inertial), float(th-pos.encoder.theata)], measures)

			print("measures " + str(measures))
			self.sendData(measures)
			x = pos.encoder.x_inertial
			y = pos.encoder.y_inertial
			th = pos.encoder.theata

			dataIN = self.reciveMessage()
			self.x = dataIN[0]
			self.y = dataIN[1]
			self.th = np.radians(dataIN[2])


	def startCommsThread(self,pos):
		thr = threading.Thread(target=self.sendPKTtoMatlab, args=(pos,))
		thr.start()