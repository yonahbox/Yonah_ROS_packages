#!/usr/bin/env python3

import os
import rospy
from pathlib import Path
import paramiko
import time

home_dir = str(Path.home())
serverfolder = "/home/ubuntu/Waypoints/"

class MissionServer():
	def __init__(self):
		server_user = "ubuntu"
		server_ip = "46.137.242.90"
		server_key = home_dir + "/mission_pvt_key.pem"
		self.ssh = paramiko.SSHClient()
		self.ssh.load_system_host_keys()
		self.ssh.connect(server_ip, username=server_user, key_filename=server_key)

	def update_files(self):
		serverupdatetime = {}
		_, stdout, _ = self.ssh.exec_command("ls Waypoints", timeout=5)
		serverfiles = stdout.readlines()
		serverfiles = [i.rstrip() for i in serverfiles]
		for i in serverfiles:
			cmd = "cat Waypoints/" + i
			_, stdout, _ = self.ssh.exec_command(cmd, timeout=5)
			serverupdatetime[i] = stdout.readlines()[-1].rstrip().split()[-1]

		localfolder = home_dir + "/Waypoints/"
		localfiles = os.listdir(localfolder)
		localupdatetime = {}
		for i in localfiles:
			g = open(localfolder + i, "r")
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
		rospy.loginfo("Finished checks")

		sftp = self.ssh.open_sftp()
		for i in to_server:
			file_src = localfolder + i
			file_dest = serverfolder + i
			sftp.put(file_src, file_dest)
			if time.time()-2 < sftp.stat(file_dest).st_mtime < time.time()+2:
				rospy.loginfo("Synced %s to server" % i)

		for i in to_local:
			file_src = serverfolder + i
			file_dest = localfolder + i
			sftp.get(file_src, file_dest)
			if time.time()-2 < os.stat(file_dest).st_mtime < time.time()+2:
				rospy.loginfo("Synced %s to local" % i)

		sftp.close()

	def close(self):
		self.ssh.close()