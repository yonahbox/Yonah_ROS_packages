#!/usr/bin/env python3

import os
import rospy
from pathlib import Path
import paramiko
import time

class MissionServer():
	def __init__(self):
		home_dir = str(Path.home())
		self.localfolder = self.home_dir + "/Waypoints/"
		self.serverfolder = "/home/ubuntu/Waypoints/"
		# TODO: Remove these variables and use identifiers + public key on server
		server_user = "ubuntu"
		server_ip = "46.137.242.90"
		server_key = home_dir + "/mission_pvt_key.pem"
		self.ssh = paramiko.SSHClient()
		self.ssh.load_system_host_keys()
		try:
			self.ssh.connect(server_ip, username=server_user, key_filename=server_key, timeout=10)
		except:
			rospy.logerr("Cannot connect to AWS Server")
			self.close()

	def update_files(self):
		serverupdatetime = {}
		try:
			_, stdout, _ = self.ssh.exec_command("ls Waypoints", timeout=5)
			serverfiles = stdout.readlines()
			serverfiles = [i.rstrip() for i in serverfiles]
			for i in serverfiles:
				cmd = "cat Waypoints/" + i
				_, stdout, _ = self.ssh.exec_command(cmd, timeout=5)
				serverupdatetime[i] = stdout.readlines()[-1].rstrip().split()[-1]

			localfiles = os.listdir(self.localfolder)
			localupdatetime = {}
			for i in localfiles:
				g = open(self.localfolder + i, "r")
				localupdatetime[i] = str(g.readlines()[-1].rstrip().split()[-1])

			to_local = []
			to_server = []
			for i in localfiles:
				if i not in serverfiles:
					to_server.append(i)
				elif localupdatetime[i] > serverupdatetime[i]:
					to_server.append(i)
				elif localupdatetime[i] < serverupdatetime[i]:
					to_local.append(i)
			for i in serverfiles:
				if i not in localfiles:
					to_local.append(i)

			sftp = self.ssh.open_sftp()
			for i in to_server:
				file_src = self.localfolder + i
				file_dest = self.serverfolder + i
				sftp.put(file_src, file_dest)
				if time.time()-2 < sftp.stat(file_dest).st_mtime < time.time()+2:
					rospy.loginfo("Synced %s to server" % i)

			for i in to_local:
				file_src = self.serverfolder + i
				file_dest = self.localfolder + i
				sftp.get(file_src, file_dest)
				if time.time()-2 < os.stat(file_dest).st_mtime < time.time()+2:
					rospy.loginfo("Synced %s to local" % i)
			
			rospy.loginfo("Files synced")
			sftp.close()
		
		except:
			rospy.logerr("Sync failed. Try another method")

	def close(self):
		self.ssh.close()
