# Despatcher

## Overview

Despatcher nodes act as a relay agent between the three links and mavros (on the aircraft) or the RQt GUI (on the Ground Control Station)

## Installation

If you haven't done so already, clone this repository and check out onto the branch on which this Readme is located. Then initialize the OGC workspace:

```
cd ogc_ws
catkin_make
```

Create the sms package, but first move out the existing despatcher folder (catkin will throw an error when it tries to create a package but the package folder already exists). After package creation, move everything in the original sms folder back into the newly created package:

```
cd src/
mv despatcher ~/
catkin_create_pkg despatcher std_msgs rospy
mv ~/despatcher/* ./despatcher/
rm -r ~/despatcher
```

After this step, the despatcher nodes, launchfiles, CMakelists.txt and package.xml should be in the package folder. Navigate back to the ogc workspace and rerun catkin_make:

```
cd ../
catkin_make
```

Finally, source the `setup.bash` file of the ogc workspace devel folder in your `.bashrc` file, and reload the `bashrc`

## Usage

Before running this package, make sure you are connected into the RUT cellular router wifi, and the ssh keypairs of the device running this package are copied into the router (see [this user guide](https://wiki.teltonika-networks.com/view/SSH_RSA_key_authentication_(Linux)) on how to copy your ssh keypairs into the router)

**Testing**

Currently, the despatcher nodes only interact with the sms link, but provisions have been made to easily expand to the other two links (telegram and SBD)

To test the air side sms link, run `sms_airtest.launch` in the sms package. In a separate terminal, launch mavros (to-do: Elaborate on this)

To test the ground side sms link, run `sms_gndtest.launch` in the sms package

**Opeational usage**

(to-do. This package is not yet operational)

## Nodes

All nodes are located in the `src` folder

### air_despatcher

Serves as the relay between mavros and the three telemetry links on the aircraft

**Subscribed topics**

* `ogc/from_sms`: Topic through which air_despatcher receives incoming G2A messages from the sms_link node
* `ogc/from_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/from_telegram`: Corresponding topic for Telegram Cellular Data link
* `mavros/*`: Family of mavros topics which air_despatcher subscribes to in order to receive data from aircraft

**Published Topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes A2G messages that are to be sent out as SMS
* `ogc/to_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/to_telegram`: Corresponding topic for Telegram Cellular Data link

**Parameters**

* `interval_1`: Short time interval between SMS regular payload
* `interval_2`: Long time interval between SMS regular payload

(To-do: Add more time intervals to cater for SBD and Telegram latencies)

### gnd_despatcher

Serves as the relay between RQt GUI and the three telemetry links on the Ground Control Station

**Subscribed topics**

* `ogc/from_sms`: Topic through which gnd_despatcher receives incoming A2G messages from the sms_link node
* `ogc/from_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/from_telegram`: Corresponding topic for Telegram Cellular Data link
* `ogc/from_despatcher/regular`: Topic through which gnd_despatcher publishes regular payload messages to RQT
* `ogc/from_despatcher/ondemand`: Topic through which gnd_despatcher publishes on-demand and miscellaneous messages to RQT

**Published Topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes G2A messages that are to be sent out as SMS
* `ogc/to_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/to_telegram`: Corresponding topic for Telegram Cellular Data linkic
* `ogc/to_despatcher`: Topic through which gnd_despatcher receives G2A commands from RQT

**Parameters**

* `interval_1`: Short time interval between SMS regular payload
* `interval_2`: Long time interval between SMS regular payload

## Msgs

All custom ROS messages are located in the `msg` folder

**RegularPayload**: Stores the regular payload format for A2G message

## Message/Command formats

**Air-to-Ground messages/payloads**

Regular payload format:

* Airspeed
* Altitude
* Armed Status
* Groundspeed
* Latitude
* Longitude
* Throttle level
* Waypoint reached
* VTOL status

**Ground-to-Air commands**

Available G2A commands:

* ping: Receive one-off messages (i.e. on-demand payloads)
    * "ping": Receive a one-off regular payload from the aircraft
    * "ping mode": Get an update on flight mode of aircraft
    * "ping Statustext": Receive the latest status text from the aircraft
* sms: Handle regular payloads from the aircraft
    * "sms true": Request regular payloads (default to long intervals) from aircraft
    * "sms false*: Deactivate regular payload service
    * "sms short": Change intervals in between regular payload to short intervals
    * "sms long": Change intervals in between regular payload to long intervals
* statustext: Handle status texts from the aircraft
    * "statustext true": Request regular status text updates from aircraft
    * "statustext false": Deactivate regular status text updates from aircraft
* "arm": Arm the aircraft
* "disarm": Disarm the aircraft
* "mode (flight mode)": Set flight mode of aircraft
* wp: Waypoint commands
    * "wp set (number)": Set target waypoint to the waypoint number specified
    * "wp load (name of waypoint file)": Load waypoint file in the home folder of the aircraft companion computer

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details