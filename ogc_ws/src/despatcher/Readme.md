# Despatcher


## Nodes

**air_despatcher**, located in `src` folder. Serves as the relay between mavros and the three telemetry links on the aircraft
**gnd_despatcher**, located in `src` folder. Serves as the relay between RQt GUI and the three telemetry links on the Ground Control Station

## Msgs

**regular_payload**, located in `msg` folder

## Installation notes

After creating the package, copy the `CMakelists.txt` and `package.xml` files in this folder to your despatcher package, before running `catkin_make` again in the `ogc_ws` workspace

## Testing

To run this with SMS, use the launch files in the sms `launch` folder for testing. Refer to the Readme in the sms package for more details

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

The air despatcher compiles A2G payloads by subscribing to the relevant mavros topics, and publishes the payloads to `ogc/to_sms`, `ogc/to_sbd` and `ogc/to_telegram` topics as strings

The ground despatcher receives A2G payloads by subscribing to `ogc/from_sms`, `ogc/from_sbd` and `ogc/from_telegram` topics. For regular payloads, they are decoded and published to `ogc/from_despatcher/regular` topic as a `regular_payload` msg. All other incoming messages will be received and published to `ogc/from_despatcher/ondemand` as strings

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

The ground despatcher receives A2G commands by subscribing to `ogc/to_despatcher` topic. It publishes these commands as strings to `ogc/to_sms`, `ogc/to_sbd` and `ogc/to_telegram`.

The air despatcher receives G2A commands by subscribing to `ogc/from_sms`, `ogc/from_sbd` and `ogc/from_telegram` topics as strings. It then forwards these commands to the autopilot by subscribing to relevant mavros services

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details