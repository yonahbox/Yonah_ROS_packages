# air_data

Package for handling Yonah's Data (Tech) telemetry, which is streamed via an AWS EC2 Server

# Nodes

**air_data**

Handles forwarding of MAVlink stream from MAVROS to netcat and finally to the AWS Server, and vice-versa.

Parameters:
* aircraft: Aircraft number
* svr_name: Hostname of AWS Server (e.g. "ubuntu")
* svr_ip: IP Address of AWS Server

@TODO: Insert a diagram showing Data Link flow

# Launchfiles

**air_data.launch**: Launches air data node. This should be used in conjunction with the `air.launch` file that is provided in the `launch` folder of this repository

**air_data_standalone.launch**: Launches air data node as a standalone node

# Usage

This package should be placed in the aircraft's Beaglebone single-board computer

1. Ensure that the AWS server is up and running
2. While ssh-ed into the Beaglebone, launch the `air.launch` file that is provided in the `launch` folder of this repository
3. Connect to the AWS using the ground scripts (@TODO: Insert link)