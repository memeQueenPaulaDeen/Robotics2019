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
	x_cm = 0
	y_cm = 0
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

	def sendDeltasAndLidarData(self, pos):
		cm2mm = 10
		mm2cm = .1

		x = pos.encoder.x_inertial*cm2mm
		y = pos.encoder.y_inertial*cm2mm
		th = pos.encoder.theata


		while True:

			measures = self.lidar.measures
			measures = np.append([float(x-pos.encoder.x_inertial*cm2mm), float(y-pos.encoder.y_inertial*cm2mm), float(th-pos.encoder.theata)], measures)

			delx = float(pos.encoder.x_inertial*cm2mm-x)
			dely = float(pos.encoder.y_inertial*cm2mm-y)
			delth = float(pos.encoder.theata - th)

			measures = np.append([delx, dely, delth], measures)

			print("measures " + str(measures))
			self.sendData(measures)
			x = pos.encoder.x_inertial*cm2mm
			y = pos.encoder.y_inertial*cm2mm
			th = pos.encoder.theata

			dataIN = self.reciveMessage()
			self.x = dataIN[0]*mm2cm
			self.y = dataIN[1]*mm2cm
			self.th = np.radians(dataIN[2])


	def startCommsThread(self,pos):
		thr = threading.Thread(target=self.sendDeltasAndLidarData, args=(pos,))
		thr.start()

	def p2t(self, x, y, th):
		return np.array([[np.cos(th), -np.sin(th), x], [np.sin(th), np.cos(th), y], [0, 0, 1]])

	def t2p(self, T):
		return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])