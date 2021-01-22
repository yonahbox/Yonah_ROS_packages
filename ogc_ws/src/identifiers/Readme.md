# Identifiers

The Identifiers package provides a centralised area to store key identifying information (hence the name "Identifiers") of all devices (aircrafts, ground stations and admin servers)

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

There is no dedicated launch file to operate this package independently, use the `ogc_airtest.launch` and `ogc_gndtest.launch` files in the root launch folder of this repository to launch the identifiers server alongside all other OGC packages/nodes.

## Nodes

The main node is the identifiers_server. It provides services for other nodes to obtain the key identifying info (e.g. phone number, Rockblock Serial) of other devices.

The node does not subscribe or publish to any topics.

See the [Identifiers Wiki](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Identifiers#identifiers_server) for the list of provided services

## Other files

Custom messages used for specifying identifiers info are located in the `msg` directory

All service files are located in the `srv` directory

The `identifier.py` module, which handles low-level identifier work (e.g. parsing of the json file) is located in the `scripts` directory.

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details.
