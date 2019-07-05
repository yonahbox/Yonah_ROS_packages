#!/usr/bin/env python

import time
import socket
import multiprocessing
import subprocess

class AWS:

	def __init__(self):		

		self.s_air = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s_air.bind(('localhost', 4000))
		self.s_ground = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s_ground.bind(('localhost', 4001))		
		self.air_conn = ''
		self.air_addr = ''
		self.ground_conn = ''
		self.ground_addr = ''
		self.air_ack = multiprocessing.Queue()
		self.ground_ack = multiprocessing.Queue()
		self.status = []

		while True:
			self.per_second()
			time.sleep(1)

	def air_sock_ping(self):
		
		self.s_air.listen(1)		
		self.air_conn, self.air_addr = self.s_air.accept()
		self.air_ack.put(self.air_conn.recv(4096))
		self.air_conn.send(str(self.status))

	def ground_sock_ping(self):

		self.s_ground.listen(1)		
		self.ground_conn, self.ground_addr = self.s_ground.accept()
		self.ground_ack.put(self.ground_conn.recv(4096))
		self.ground_conn.send(str(self.status))

	def per_second(self):		

		self.air_ack = multiprocessing.Queue()
		self.ground_ack = multiprocessing.Queue()
	
		self.processes = [multiprocessing.Process(target=self.air_sock_ping), multiprocessing.Process(target=self.ground_sock_ping)]

		
		for p in self.processes:
			p.start()

		for p in self.processes:
			p.join(0.5)
		
		for p in self.processes:
			p.terminate()	

		self.status = []
			
		try:		
			self.status.append(self.air_ack.get(timeout=0.1))			
		except:
			self.status.append('')
			
		
		try:		
		 	self.status.append(self.ground_ack.get(timeout=0.1)) 		
		except:
			self.status.append('')

		if ("AIR" in self.status) and (("GROUND" in self.status) or ("GROUNDNETCAT" in self.status)):	
			print "Air-Server-Ground Established"
		
		elif ("AIR" in self.status) and not (("GROUND" in self.status) or ("GROUNDNETCAT" in self.status)):
			print "Air-Server Established, Server-Ground Connection Down, Please Reconnect"				

		elif not ("AIR" in self.status) and (("GROUND" in self.status) or ("GROUNDNETCAT" in self.status)):
			print "Server-Ground Established, Air-Server Connection Down, Please Reconnect"	

		else:
			print "SSH Connection Lost"

	def socket_terminate(self):
		self.s_air.shutdown()
		self.s_air.close()
		self.s_ground.shutdown()
		self.s_ground.close()


aws = AWS()

try:
	while True:
		aws.per_second
		time.sleep(1)
	
except KeyboardInterrupt:
	print "Terminating Connection..."
	aws.socket_terminate()
