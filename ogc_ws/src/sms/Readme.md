# SMS (Ops)

## Overview

ROS Package to handle Yonah's SMS Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

## Installation

If you haven't done so already, clone this repository and check out onto the branch on which this Readme is located. Then initialize the OGC workspace:

```
cd ogc_ws
catkin_make
```

Create the sms package, but first move out the existing sms folder (catkin will throw an error when it tries to create a package but the package folder already exists). After package creation, move everything in the original sms folder back into the newly created package:

```
cd src/
mv sms ~/
catkin_create_pkg sms std_msgs rospy
mv ~/sms/* ./sms/
rm -r ~/sms
```

After this step, the sms nodes, launchfiles, CMakelists.txt and package.xml should be in the package folder. Finally, navigate back to the ogc workspace and rerun catkin_make:

```
cd ../
catkin_make
```

Source the `setup.bash` file of the ogc workspace devel folder in your `.bashrc` file, and reload the `bashrc`

Change the `client_phone_no` parameter in the launch files (located in the `launch` folder) to the phone number of the receipient. For example, if this package is deployed on the aircraft, specify the phone number of the GCS, and vice-versa. Be sure the include the country code of the number (e.g. +6512345678, instead of 12345678))

Make sure that a text file containing whitelisted phone numbers (with the title `whitelist.txt`) is located in `src` directory

* When specifying the phone numbers, be sure to include the country code of the number
* Example file, containing one Singapore and one Malaysian phone number (`whitelist.txt`):

```
+6512345678
+60123456789
```

## Usage

Before running this package, make sure you are connected into the RUT cellular router wifi, and the ssh keypairs of the device running this package are copied into the router (see [this user guide](https://wiki.teltonika-networks.com/view/SSH_RSA_key_authentication_(Linux)) on how to copy your ssh keypairs into the router)

**Testing**

To test the air side sms link, run `sms_airtest.launch`. In a separate terminal, launch mavros (to-do: Elaborate on this)

To test the ground side sms link, run `sms_groundtest.launch`

**Opeational usage**

(to-do. This package is not yet operational)

## Launch files

All launch files are located in the launch folder

* **sms_airtest.launch**: Launch file to test air side sms link. Launches sms link with the air despatcher node
* **sms_gndtest.launch**: Launch file to test ground side sms link. Launches sms link with the ground despatcher node

See each launch file for a description of their arguments

## Nodes

All nodes are located inside the `src` folder

### sms_link

Interacts with the cellular router to send and receive SMS messages

**Subscribed topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes messages that are meant to be sent out as SMS

**Published Topics**

* `ogc/from_sms`: Topic through which the sms_link node publishes messages that it had received as SMS

**Parameters**

* `router_hostname`: Hostname and IP address of cellular router
* `client_phone_no`: Phone number of client (if this package is running on the ground, then the client is the aircraft, and vice versa)
* `whitelist`: Location of the text file containing whitelisted phone numbers, relative to the sms package folder

## Modules

Custom python modules are located in the `scripts` folder

**RutOS**: Functions that interact with the RUT router's command-line interface (to access sms and gps commands)

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details