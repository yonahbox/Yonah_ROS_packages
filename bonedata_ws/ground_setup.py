import time
import subprocess
from subprocess import Popen, PIPE

# SSH Initialisation
probe = subprocess.Popen(['pidof' , 'ssh'], stdout=PIPE)
ssh_list = probe.stdout.read()
arg = 'kill -9 ' + ssh_list
subprocess.Popen([arg], stdout=PIPE, stderr=PIPE, shell=True)
print "Killed existing SSH-related processes with PID: " + ssh_list
ssh_connected = False
ssh_attempt = subprocess.Popen(['bash', 'ground_ssh_connection.sh'], stdout=PIPE)
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

# MAVPROXY Initialisation
subprocess.Popen(['bash', 'launch_GCS.sh'])

# NETCAT Initialisation
if ssh_connected == True:
	probe = subprocess.Popen(['pidof' , 'netcat'], stdout=PIPE)
	netcat_list = probe.stdout.read()
	arg = 'kill -9 ' + netcat_list
	subprocess.Popen([arg], stdout=PIPE, stderr=PIPE, shell=True)
	print "Killed existing NETCAT-related processes with PID: " + netcat_list 	
	print "Attempting NETCAT Connection..."
	subprocess.Popen(['bash', 'ground_netcat_init.sh'])
	
