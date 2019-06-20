import time
import subprocess
from subprocess import Popen, PIPE
import rosnode
import paramiko
import sshtunnel

def check_ros_nodes(nodes):
	if nodes == rosnode.get_node_names():
		return True
	else:
		return False

class SSH:

	def __init__(self):
		
		self.__server_address = 'ec2-18-138-24-228.ap-southeast-1.compute.amazonaws.com'
		self.__username = 'ubuntu'
		self.__pkey = 'YonahAWSPrivateKey-8May19.pem'

	def ssh_test_connection(self):
		
		for attempts in range(5):
			if __ssh.get_transport().is_active():
				print "SSH Link Connected"
				return True
			else:
				print "Trying again..."
				time.sleep(0.5)				
				continue
		print "SSH Link Failed"
		return False
	
	def ssh_attempt_connection(self):
				
		self.__ssh = paramiko.SSHClient()
		self.__ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		self.__ssh.connect(__server_address, username=__username, key_filename=__pkey)
		
		forward_tunnel(4000, __server_address, 4000, self.__ssh)

	def ssh_attempt_reconnection(self, counter):

		self.__ssh.close()
		self.ssh_attempt_connection()
		if (ssh_test_connection == False):
			if (counter is not 0):			
				print "Attempting reconnection..."			
				self.ssh_attempt_reconnection(self, counter - 1)
				return False			
			else:
				print "Connection Failed"
				return False
		else:
			print "Connection Resumed!"
			return True
			

ssh = SSH()
ssh_link = False
alert = False
		
while True:
	
	if (ssh.ssh_test_connection() == False):
		ssh_link = False
		if (ssh.ssh_attempt_reconnection(5) = True):
			ssh_link = True
		else:
			alert = True
	sleep(2)
			
			
		
		
	


# ROS Initialisation
if ('/rosout' in rosnode.get_node_names()) and ('/mavros' not in rosnode.get_node_names())
subprocess.Popen(['rosnode', 'kill', '-a'])
print "Killed ROS"
subprocess.Popen(['bash', 'launch_ros.sh'])

# SSH Initialisation
probe = subprocess.Popen(['pidof' , 'ssh'], stdout=PIPE)
ssh_list = probe.stdout.read()
arg = 'kill -9 ' + ssh_list
subprocess.Popen([arg], stdout=PIPE, stderr=PIPE, shell=True)
print "Killed existing SSH-related processes with PID: " + ssh_list
ssh_connected = False
ssh_attempt = subprocess.Popen(['bash', 'air_ssh_connection.sh'], stdout=PIPE)
#Test for SSH Connection
while ssh_connected == False:
	probe = subprocess.Popen(['netstat', '-t'], stdout=PIPE)
	network_list = probe.stdout.readlines()
	for i in network_list:	
		if ('ec2-18-138-24-228' in i) and ('ESTABLISHED' in i):
			print "Success!"
			print "Connection details: "
			print i
			ssh_connected = True
			break
	if ssh_connected == False:
		print "Trying again..."
		time.sleep(0.1)

while (not check_ros_nodes(['/rosout', '/mavros'])) or (ssh_connected == False):
	continue

# NETCAT Initialisation
probe = subprocess.Popen(['pidof' , 'netcat'], stdout=PIPE)
netcat_list = probe.stdout.read()
arg = 'kill -9 ' + netcat_list
subprocess.Popen([arg], stdout=PIPE, stderr=PIPE, shell=True)
print "Killed existing NETCAT-related processes with PID: " + netcat_list 	
print "Attempting NETCAT Connection..."
subprocess.Popen(['bash', 'air_netcat_init.sh'])


	
