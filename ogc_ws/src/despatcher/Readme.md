# Despatcher

## Overview

Despatcher nodes act as a relay agent between the three links and mavros (on the aircraft) or the RQt GUI (on the Ground Control Station)

In addition to the despatchers, this package also contain two other nodes: Feedback Timeout and Switcher Nodes. They are placed in the despatcher package as they are centralised nodes receiving data from all three links, similar to how despatcher nodes are centralised.

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

Before running this package, make sure your device:

* Is connected into the RUT cellular router wifi, and the ssh keypairs of the device running this package are copied into the router (see [this user guide](https://wiki.teltonika-networks.com/view/SSH_RSA_key_authentication_(Linux)) on how to copy your ssh keypairs into the router)
* Has the Rockblock 9603 satellite modem connected, if running/testing the SBD link

**Testing**

The despatcher nodes can be tested with any one of the three links. To test each link, use the launch files in the respective link packages (sbd, sms, tele)

**Opeational usage**

To run the despatcher node and links on the air side, go to the root launch folder of this repository and run `ogc_airtest.launch`

To run the despatcher node and links on the ground side, go to the root launch folder of this repository and run `ogc_gndtest.launch`

## Nodes

All nodes are located in the `src` folder

### air_despatcher

Serves as the relay between mavros and the three telemetry links on the aircraft

**Subscribed topics**

* `ogc/from_sms`: Topic through which air_despatcher receives incoming G2A messages from the sms_link node
* `ogc/from_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/from_telegram`: Corresponding topic for Telegram Cellular Data link
* `ogc/statustext`: Receive condensed statustexts from Statustext Handler Node
* `ogc/from_switcher`: Receive link switch commands from switcher node
* `ogc/files_modified`: Receive update on modified files from syncthing node
* `ogc/ack_tele_file`: Used for file syncing over telegram
* `ogc/to_despatcher/error`: Receive error messages
* `ogc/from_rff`: Receive commands to move to the next mission from RFF node
* `ogc/to_rff`: Receive the armed state of aircraft to determine whether to pause or resume syncthing
* `ogc/identifiers/valid_ids`: Receive the list of authorised device IDs if it is changed
* `mavros/*`: Family of mavros topics which air_despatcher subscribes to in order to receive data from aircraft

**Published Topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes A2G messages that are to be sent out as SMS
* `ogc/to_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/to_telegram`: Corresponding topic for Telegram Cellular Data link
* `ogc/to_rff`: Send mission commands to RFF node
* `ogc/files/syncthing`: Communication with syncthing node

**Subscribed Services**

* `identifiers/self/self_id`: Get the aircraft's own IDs
* `identifiers/get/valid_ids`: Get the list of authorised device IDs
* `mavros/*`: Family of mavros services that act on G2A commands

**Parameters**

* `interval_1`: Short time interval between regular payload
* `interval_2`: Medium time interval between regular payload
* `interval_3`: Long time interval between regular payload
* `waypoint_folder`: Folder in which the aircraft waypoints and mission files are stored

### gnd_despatcher

Serves as the relay between RQt GUI and the three telemetry links on the Ground Control Station

**Subscribed topics**

* `ogc/from_sms`: Topic through which gnd_despatcher receives incoming A2G messages from the sms_link node
* `ogc/from_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/from_telegram`: Corresponding topic for Telegram Cellular Data link
* `ogc/to_despatcher`: Receive G2A commands from RQT
* `ogc/identifiers/valid_ids`: Receive the list of authorised device IDs if it is changed
* `ogc/from_switcher`: Receive link switch commands from switcher node

**Published Topics**

* `ogc/to_sms`: Topic through which the despatcher node publishes G2A messages that are to be sent out as SMS
* `ogc/to_sbd`: Corresponding topic for Iridium Short-Burst-Data (SBD) link
* `ogc/to_telegram`: Corresponding topic for Telegram Cellular Data linkic
* `ogc/from_despatcher/regular`: Publish regular payload messages to RQT
* `ogc/from_despatcher/statustext`: Publish statustext messages to Statustext Handler Node
* `ogc/from_despatcher/ondemand`: Publish miscellaneous messages to RQT
* `ogc/to_telegram/file`: Used for file syncing over telegram
* `ogc/to_timeout`: Send the status of outgoing messages to the feedback_timeout module (for Feedback Message Design)

**Subscribed Services**

* `identifiers/self/self_id`: Get the aircraft's own IDs
* `identifiers/get/valid_ids`: Get the list of authorised device IDs

**Parameters**

* `interval_1`: Short time interval between heartbeats
* `interval_2`: Medium time interval between heartbeats
* `interval_3`: Long time interval between heartbeats

### feedback_timeout

Provide feedback to the ground operator on whether a G2A command succeeed or failed in getting sent. Present only on ground side. See the [Feedback Message documentation](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Feedback) for more details.

**Subscribed Topics**

* `ogc/to_timeout` and `ogc/to_despatcher`: Receive feedback on outgoing messages from other nodes (e.g. despatcher, link nodes)

**Published Topics**

* `ogc/feedback_to_rqt`: Publish feedback on outgoing messages to the RQT to be displayed to the ground user

### switcher

Handles link switching on both air and ground side. See [link switching documentation](https://github.com/yonahbox/Yonah_ROS_packages/wiki/LinkSwitch) for more details.

**Subscribed Topics**

* `ogc/from_sms`: Topic to receive incoming messages (A2G, G2A) from the sms_link node
* `ogc/from_telegram`: Corresponding topic for Telegram Cellular Data link
* `ogc/to_switcher_tele`: Receive **internal** messages on the status of the Telegram link (e.g. whether timeout has occured when sending a message over Telegram
* `ogc/to_switcher_sms`: Receive **internal** messages on the status of the SMS link (e.g. whether error message was received from router when sending an SMS)
* `ogc/identifiers/valid_ids`: Receive the list of authorised device IDs if it is changed

**Published Topics**

* `ogc/from_switcher`: Publish link switch updates (e.g. which link to switch to when current link is declared as failed)

**Subscribed Services**

* `identifiers/get/valid_ids`: Get the list of authorised device IDs

**Parameters**

* `router_username`: Username of Cellular Router
* `router_ip`: IP Address of Cellular Router

## Msgs

All custom ROS messages are located in the `msg` folder

**RegularPayload**: Stores the regular payload format for A2G message

**LinkMessage**: Stores the format used for an outgoing message (data, ID of receipient, UUID of message)

## Message/Command formats

**Air-to-Ground messages/payloads**

Regular payload is sent from air to ground at specified intervals. See the [regular payload](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Regular-Payload-Design) for more details

**Ground-to-Air commands**

Available G2A commands can be found in the [G2A Wiki Page](https://github.com/yonahbox/Yonah_ROS_packages/wiki/G2A)

In addition to G2A commands, heartbeats are also sent at regular intervals (similar to regular payload but without the data) from ground to air. This is for [link switching](https://github.com/yonahbox/Yonah_ROS_packages/wiki/LinkSwitch) purposes.

**Message Headers**

Each A2G and G2A message follows a predefined header format specified in this [Wiki page](https://github.com/yonahbox/Yonah_ROS_packages/wiki/G2A)

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
