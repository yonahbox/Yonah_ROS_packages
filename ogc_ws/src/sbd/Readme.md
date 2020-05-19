# SBD

ROS Package to handle Yonah's Iridum Short-Burst-Data (SBD) Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

## Installation

If you haven't done so already, clone this repository and check out onto the branch on which this Readme is located. Then initialize the OGC workspace:

```
cd ogc_ws
catkin_make
```
Source the `setup.bash` file of the ogc workspace devel folder in your `.bashrc` file, and reload the `bashrc`

## Usage

To-do

**Testing**

**Opeational usage**

## Launch files

All launch files are located in the launch folder

To-do

## Nodes

All nodes are located inside the `src` folder

### sbd_air_link

Send and receive SBD msgs for the aircraft

To-do: Add topics/svcs for this node

### sbd_gnd_link

Send and receive SBD msgs for GCS

To-do: Add topics/svcs for this node

# Modules

Custom python modules are located in the `scripts` folder

**rockBlock**: Handles serial communication with Rockblock modem. Adopted and modified from Makersnake's [pyRockblock] (https://github.com/MakerSnake/pyRockBlock), and re-released under the GNU-GPL-v3 license

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details