# RFF

The RFF monitors mission-related hardware (button, buzzer) that are key to allowing the aircraft to takeoff for a new mission/return trip after landing at a delivery site.

See the [RFF Wiki](https://github.com/yonahbox/Yonah_ROS_packages/wiki/RFF) for more details.

## Node

The `RFF` node is in the `src/` directory

The node connects to the router via ssh and continuously checks the state of the button.

**Subscribed Topics**

* `mavros/state`: Check whether the aircraft is armed/disarmed
* `ogc/to_rff`: Get the hop number from despatcher (to determine whether the aircraft will takeoff for the next set of waypoints, or fly a return journey)

**Published Topics**

* `ogc/from_rff`: Publish `mission next` command to the despatcher when the button is pressed for more than 5 seconds

**Parameters**

* `router_username`: Username of cellular router
* `router_ip`: IP address of cellular router

# License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
