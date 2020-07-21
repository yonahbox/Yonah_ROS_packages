# SBD

ROS Package to handle Yonah's Iridum Short-Burst-Data (SBD) Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

The node interfaces with a Rockblock 9603, connected to the main system via a FTDI-USB cable.

To-do: Add more details on 9603 setup and comms architecture (Rockblock 2 Rockblock, Web Server)

## Installation

If you haven't done so already, clone this repository and check out onto the branch on which this Readme is located. Then initialize the OGC workspace:

```
cd ogc_ws
catkin_make
```
Source the `setup.bash` file of the ogc workspace devel folder in your `.bashrc` file, and reload the `bashrc`

## Usage

Before running this package, ensure the following;

* Your system (companion computer, GCS) is connected to the Rockblock modem via the FTDI-USB cable, with the modem having a clear view of the sky
* Your web server is running with the neccessary scripts in your server cgi-bin folder (to-do: Guide on how to set up the scripts + apache service)
* A `login.txt` file containing the neccessary login credentials is placed in the `src` folder. The file should have: (1) URL of your web sever script, (2) IMEI number of the Rockblock modem you are connected to, (3) Email used to login to Rock Seven Web Services, (4) Password used to login to Rock Seven Web Services. For example:

```
http://ec2-1-2-3-4.ap-southeast-1.compute.amazonaws.com/cgi-bin/testscript.py
123456789012345
somebody@gmail.com
fakepassword
```

**Testing**

To test the air side sbd link, run `sbd_airtest.launch`. In a separate terminal, launch mavros (to-do: Elaborate on this)

To test the ground side sbd link, run `sbd_gndtest.launch`

**Opeational usage**

(to-do. This package is not yet operational)

## Launch files

All launch files are located in the launch folder

* **sbd_airtest.launch**: Launch file to test air side sbd link. Launches sbd link with the air despatcher node
* **sbd_gndtest.launch**: Launch file to test ground side sbd link. Launches sbd link with the ground despatcher node
* **sbd_test.launch**: Tests either the air or ground side sbd link as a standalone

See each launch file for a description of their arguments

Note: The `interval_1` and `interval_2` arguments in the launch files are currently not used

## Nodes

All nodes are located inside the `src` folder

### sbd_air_link

Send and receive SBD msgs for the aircraft

**Subscribed topics**

* `ogc/to_sbd`: Topic through which the despatcher node publishes messages that will be sent out via SBD

**Published Topics**

* `ogc/from_sbd`: Topic through which the sbd_link node publishes messages that it had received via SBD

**Parameters**

* `interval`: Time interval between each SBD mailbox check in seconds
* `own_serial`: Serial number of the Rockblock connected to the aircraft. **Note that this is not the IMEI number**
* `client_serial`: Serial number of the Rockblock connected to the GCS
* `portID`: Serial port of the aircraft companion computer where the Rockblock is connected to (e.g. dev/ttyUSB0)

### sbd_gnd_link

Send and receive SBD msgs for GCS. Inherits most of the code from **sbd_air_link** node

**Subscribed topics**

* `ogc/to_sbd`: Topic through which the despatcher node publishes messages that will be sent out via SBD

**Published Topics**

* `ogc/from_sbd`: Topic through which the sbd_link node publishes messages that it had received via SBD

**Parameters**

* `interval`: Time interval between each SBD mailbox check in seconds
* `own_serial`: Serial number of the Rockblock connected to the GCS. **Note that this is not the IMEI number**
* `client_serial`: Serial number of the Rockblock connected to the aircraft
* `portID`: Serial port of the GCS where the Rockblock is connected to (e.g. dev/ttyUSB0)
* `credentials`: Location of the login.txt file containing the required login info to send and receive messages via the web server method

# Modules

Custom python modules are located in the `scripts` folder

**rockBlock**: Handles serial communication with Rockblock modem. Adopted and modified from Makersnake's [pyRockblock] (https://github.com/MakerSnake/pyRockBlock), and re-released under the GNU-GPL-v3 license

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details