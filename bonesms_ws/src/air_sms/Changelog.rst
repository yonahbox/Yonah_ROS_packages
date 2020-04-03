=============================
Changelog for package air_sms
=============================

1.0
------------------

- Initial Release
- Messages sent from aircraft during regular SMS updates (name displayed in SMS, units):
    - Airspeed ("AS", m/s)
    - Altitude ("alt", m)
    - Arm status ("Arm", bool)
    - AWS Data Link Telemetry Status ("AWS", bool)
    - Climb Rate ("cmb", m/s)
    - GPS Lat-Lon ("lat" and "lon", lat-lon)
    - Groundspeed ("GS", m/s)
    - Heading ("yaw", deg)
    - Mode ("mode", string)
- Available commands (name of command): 
    - Arm/Disarm aircraft ("arm" or "disarm")
    - Change flight mode ("mode <flight mode>")
    - Activate/Deactivate regular SMS updates from aircraft ("sms true" or "sms false")
    - Command aircraft to send a single SMS update ("ping")
    - Request regular SMS updates to be done in long/short intervals ("sms long" or "sms short")
- Other features:
    - List of whitelisted phone numbers is stored in "whitelist.txt", located in the same folder as SMS nodes
    - Ground Operator phone number is specified in launch file


2.0
------------------

Redefine SMS 2.0 as SMS(Tech) 2.0; this package will be used to handle SMS Telemetry for technical development. A separate package will be created for operational purposes

- Definition updates:
    - Messages sent by aircraft to ground control are defined as air-to-ground (A2G)
    - Messages sent by ground control to aircraft are defined as ground-to-air (G2A) 
    - Regular A2G SMS updates are defined as "Regular Payload".
    - Special A2G data that are sent when ground operator requests for it are defined as "On-Demand"
    - Text messages that normally appear on console of GCS are defined as "statustext"
- Add the following available G2A commands (name of command):
    - Request for any of the on-demand payload data: ("ping <on-demand data>", e.g. "ping mode" to get flight mode)
    - Set target waypoint ("wp set <waypoint number")
    - Load a mission file residing in the companion computer: ("wp load <path to waypoint file>")
- Fixed a bug in "sms short" and "sms long" commands
- Add capability for air_sms to be tested on SITL
- Align air_sms package with ROS Best Practices
- Break out RutOS commands into its separate module
- Aircraft now acknowledges incoming G2A commands by sending a reply A2G SMS with the format ("ACK: <G2A command>")
- Break out router hostname into a ROSlaunch parameter
- Add feature where alerts are sent when special events occur (e.g. when vibrations exceed certain value)
- Compact regular payload length
- Timestamp is now added to all A2G messages
- Remove requirement to have a newline at the end of the whitelist.txt file

- New Regular A2G Payload format (name displayed in SMS, units):
    - Arm status ("Arm", bool)
    - Airspeed ("AS", m/s)
    - Gndspeed ("GS", m/s)
    - Throttle ("thr", scaled between 0.0 to 1.0)
    - Altitude relative to home ("alt", m)
    - Latitude ("lat" - )
    - Longitude ("lon", - )
    - Status of AWS Data Telemetry Node ("AWS", bool)
    - Waypoint that has been reached ("wp", integer)
    - Whether aircraft is in VTOL mode ("VTOL", bool)
- New On-demand A2G payload data:
    - Mode ("mode", String)
    - Status Text Messages ("msg", String)
    - Vibration levels ("vibe", m/s/s)
    - Clipping Events ("clipping", integer)