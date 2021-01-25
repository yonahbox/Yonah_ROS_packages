# SMS (Ops)

## Overview

ROS Package to handle Yonah's SMS Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

Before running this package, make sure you are connected into the RUT cellular router wifi, and the ssh keypairs of the device running this package are copied into the router (see [this user guide](https://wiki.teltonika-networks.com/view/SSH_RSA_key_authentication_(Linux)) on how to copy your ssh keypairs into the router)

**Testing**

To test the air side sms link, run `sms_airtest.launch`. In a separate terminal, launch mavros.

To test the ground side sms link, run `sms_gndtest.launch`

**Operational usage**

To run the sms link on the air side, go to the root launch folder of this repository and run `ogc_airtest.launch`

To run the sms link on the ground side, go to the root launch folder of this repository and run `ogc_gndtest.launch`

## Launch files

All launch files are located in the launch folder

* **sms_airtest.launch**: Launch file to test air side sms link. Launches sms link with the air despatcher node
* **sms_gndtest.launch**: Launch file to test ground side sms link. Launches sms link with the ground despatcher node

See each launch file for a description of their arguments

## Nodes

All nodes are located inside the `src` folder

### sms_link

Interacts with the cellular router to send and receive SMS messages. See the SMS [Wiki Page](https://github.com/yonahbox/Yonah_ROS_packages/wiki/SMS-Link) for detailed information on how the link operates.

**Subscribed topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes messages that are meant to be sent out as SMS

**Published Topics**

* `ogc/from_sms`: Topic through which the sms_link node publishes messages that it had received as SMS
* `ogc/to_switcher_sms`: Topic through which the sms_link node publishes updates on the router connection status. Data is used by switcher node
* `ogc/to_timeout`: Topic thorugh which the sms_link node publishes feedback on the delivery status of outgoing SMS. Data is used by the timeout node

**Subscribed Services**

* `identifiers/check/proper`: To verify that incoming messages come from valid clients
* `identifiers/get/number`: Get the phone number of the client which the node is sending messages to

**Parameters**

* `router_username`: Username of cellular router
* `router_ip`: IP address of cellular router

## Modules

Custom python modules are located in the `scripts` folder

**RutOS**: Functions that interact with the RUT router's command-line interface (to access sms and gps commands)

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
