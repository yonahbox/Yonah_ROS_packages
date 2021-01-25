# File_Sync

This package provides file syncing services, which syncs files such as waypoint, mission and identifier files across all devices. The peer-to-peer file sharing software **Syncthing** is used.

The syncthing software will be paused during certain stages of the mission for safety reasons, with a backup syncthing method via telegram used when the pause occurs.

See [file syncing](https://github.com/yonahbox/Yonah_ROS_packages/wiki/File-Syncing) and [syncthing](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Syncthing) wiki for more details

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

Before running this package, make sure you are connected into the RUT cellular router wifi so that Syncthing can access the Internet. Also ensure that syncthing is installed and running (this should already have been done during the software install process mentioned above)

Launch the node alongside all other OGC packages/nodes using the `ogc_airtest.launch` and `ogc_gndtest.launch` files in the root launch folder of this repository.

## Nodes

The syncthing node is located in the `src` folder.

**Subscribed Topics**

* `ogc/files/syncthing`: Receive commands on whether to pause or resume syncthing

**Published Topics**

* `ogc/files/connected`: Send a notification when detecting a new device connecting to the syncthing network
* `ogc/files/disconnected`: Send a notification when detecting a device exiting the syncthing network

**Subscribed Services**

* `identifiers/set/st_id`: Request to add a device's syncthing ID to the identifiers file
* `identifiers/get/syncthing_ids`: Get the list of syncthing IDs for all valid devices within the network
* `identifiers/get/id`: Get the ID of a specified device based on a given key identifying into (e.g. phone number, syncthing ID)
* `identifiers/get/all`: Get all identifying details of a specified device

**Parameters**

* `watch_dir`: Directory to the folder containing all files that are to be synced

## Other Files

The `syncthing.py` module, which handles low level interactions with syncthing, is located in the `scripts` folder

The `GetStatus` ROS service, which gets the status (pause or resume) of syncthing, is located in the `srv` folder

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
