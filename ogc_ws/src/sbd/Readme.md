# SBD

ROS Package to handle Yonah's Iridum Short-Burst-Data (SBD) Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

The node interfaces with a Rockblock 9603, connected to the main system via a FTDI-USB cable.

More details on how the SBD link works is in the [Wiki Page](https://github.com/yonahbox/Yonah_ROS_packages/wiki/SBD-Link).

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

Before running this package, ensure the following;

* Your system (companion computer, GCS) is connected to the Rockblock modem via the FTDI-USB cable, with the modem having a clear view of the sky
* Your web server is running with the neccessary scripts in your server cgi-bin folder, see [Server Installation Instructions](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation#server-side-installation)
* The identifiers file already contains the neccessary login credentials (see [Identifiers Wiki](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Identifiers#the-json-file) for more details), for example:

```
"sbd_details": {
		"rock7_username": "somebody@gmail.com",
		"rock7_password": "fakepassword",
		"svr_hostname": "ubuntu",
                "svr_ip": "127.0.0.1"
	},
```

**Testing**

To test the air side sbd link, run `sbd_airtest.launch`. In a separate terminal, launch mavros

To test the ground side sbd link, run `sbd_gndtest.launch`

**Operational Usage**

To run the sms link on the air side, go to the root launch folder of this repository and run `ogc_airtest.launch`

To run the sms link on the ground side, go to the root launch folder of this repository and run `ogc_gndtest.launch`

## Launch files

All launch files are located in the launch folder

* **sbd_airtest.launch**: Launch file to test air side sbd link. Launches sbd link with the air despatcher node
* **sbd_gndtest.launch**: Launch file to test ground side sbd link. Launches sbd link with the ground despatcher node
* **sbd_test.launch**: Tests either the air or ground side sbd link as a standalone

See each launch file for a description of their arguments

## Nodes

All nodes are located inside the `src` folder.

### sbd_air_link

Send and receive SBD msgs for the aircraft

**Subscribed topics**

* `ogc/to_sbd`: Topic through which the despatcher node publishes messages that will be sent out via SBD
* `ogc/identifiers/valid_ids`: Receive the list of authorised device IDs if it is changed

**Published Topics**

* `ogc/from_sbd`: Topic through which the sbd_link node publishes messages that it had received via SBD
* `ogc/to_timeout`: Publishes feedback on the delivery status of outgoing SBD msg. Data is used by the timeout node. **Note**: This is inherited and used by the sbd ground node, and is actually not actively used in the sbd air node (the feedback timeout node does not run on the air side)

**Subscribed Services**

* `identifiers/self/serial`: Get the Rockblock serial number of the local device
* `identifiers/get/serial`: Get the Rockblock serial number of the specified client device
* `identifiers/check/lazy`: Perform a simple verification of whether an incoming message is from a valid sender
* `identifiers/get/valid_ids`: Request for the list of authorised device IDs

**Parameters**

* `thr_server`: Whether the server or the RB-2-RB method will be used. 1 = Server, 0 = RB-2-RB
* `interval`: Time interval between each SBD mailbox check in seconds
* `portID`: Serial port of the aircraft companion computer where the Rockblock is connected to (e.g. dev/ttyUSB0)

### sbd_gnd_link

Send and receive SBD msgs for GCS. Inherits most of the code from **sbd_air_link** node

**Subscribed and Published topics**

The subscribed and published topics are the same as and inherited from sbd_air_link

**Subscribed Services**

The sbd_gnd_link inherits all services from the sbd_air_link, and uses the additional services:

* `identifiers/get/imei`: Get the Rockblock IMEI number of the specified client device
* `identifiers/check/proper`: To verify that incoming messages come from valid clients
* `identifiers/self/sbd`: Get the credentials required for accessing the admin server (for server communication)
* `identifiers/self/self_id`: Get the identifier-assigned ID of the local device

**Parameters**

The parameters are the same as and inherited from sbd_air_link

# Modules

Custom python modules are located in the `scripts` folder

**rockBlock**: Handles serial communication with Rockblock modem. Adopted and modified from Makersnake's [pyRockblock] (https://github.com/MakerSnake/pyRockBlock), and re-released under the GNU-GPL-v3 license

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
