# RQt Plugin

RQt is a software framework of ROS that implements the various GUI tools in the form of plugins. It is based on the Qt framework (https://www.qt.io/) and it enables users to create a UI interface that could communicate with ROS.

For OGC, RQt is used as a UI for the ground operator to interact with the aircraft. It runs on the ground side. See [Yonah's RQT documentation](https://github.com/yonahbox/Yonah_ROS_packages/wiki/OGC-RQt) for more details

## Installation

This package should be installed alongside the rest of Yonah's OGC. See the [Software Install](https://github.com/yonahbox/Yonah_ROS_packages/wiki/Software-Installation) page for more details.

## Usage

**Testing**

RQT can be run standalone to check its functionality. To do this, run rqt and under plugin you should be able to see Yonah plugin

```
$ rqt
```

Alternatively, you can directly run the rqt plugin without needing to access the main rqt page using

```
rqt --standalone yonah_rqt
```

**Operational Usage**

Launch RQT on the ground side using `ogc_gndtest.launch`; this launch file is located in the root launch folder of this repository.

## Nodes

The main node is `yonah_rqt`, located in `scripts` folder. It launches the RQt main window by calling `main_window.py` from the `src` folder.

**Subscribed Topics**

* `ogc/from_despatcher/regular`: Receive regular payload from despatcher
* `ogc/from_despatcher/ondemand`: Receive on-demand messages from despatcher
* `ogc/yonahtext` Receive decoded statustexts from ground statustext handler
* `ogc/files/conflict`: Get warnings of file conflicts, if any, from syncthing
* `ogc/feedback_to_rqt`: Receive updates on the delivery status of G2A commands from the feedback message node
* `mavros/*`: Receive SITL-based aircraft info from MAVROS; **this is purely for testing with SITL and will not be used in operations**

**Published Topics** 

* `ogc/to_despatcher`: Send G2A commands to the gnd despatcher

## Other files

The bulk of the source code is in the `src` folder:

* `main_window.py`: Launches the RQt main window with only key aircraft info and commands; called by the yonah_rqt node
* `aircraft_info.py`: Contains detailed info on each active aircraft
* `checklist_window.py`: Displays checklist for ground operators prior to the mission
* `command_window.py`: Contains full list of available G2A commands
* `log_window.py`: To display ROS logs for debugging purposes
* `popup_window.py`: For popup messages (e.g. warnings)
* `summary_window.py`: Part of the RQt that shows the main information about each aircraft
* `valid_id_window.py`: Spawned when rqt first starts up and waits for ground operator to key in the valid IDs. Responsible for initial setting up of OGC.
* `waypoint_window.py`: Contains waypoints and progress for each aircraft

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details

## Built With

* [ROS](https://www.ros.org/)
* [RQt](http://wiki.ros.org/rqt) 

