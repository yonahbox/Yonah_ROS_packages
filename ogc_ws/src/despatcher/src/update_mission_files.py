#!/usr/bin/env python3

import time
import rospy
from pathlib import Path
from despatcher.msg import LinkMessage
from os import listdir

class mission_updater():
	def __init__(self):
		rospy.init_node('mission_updater', anonymous=False)
		self.pub_to_despatcher = rospy.Publisher('ogc/to_despatcher', LinkMessage, queue_size = 5)
		
	def update(self):
		home_dir = str(Path.home())
		gndfolder = home_dir + "/Sync/Waypoints/"
		gndfiles = listdir(gndfolder)
		mission_msg = []
		for i in gndfiles:
			g = open(gndfolder + i, "r")
			update_time = g.readlines()[-1].rstrip().split()[-1]
			mission_msg.append(str(i) + " " + str(update_time))
		update = LinkMessage()
		update.uuid = 0
		update.id = 4
		update.data = "mission update " + " ".join(mission_msg)
		time.sleep(1)
		self.pub_to_despatcher.publish(update)

if __name__=='__main__':
	run = mission_updater()
	run.update()
