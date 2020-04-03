# air_sms

Package for handling Yonah's SMS Telemetry

## air_sms Nodes

* **SMS_Tx**: Handles sending of messages to the onboard UAV router, which will proceed to send it to Ground Control
* **SMS_Rx**: Extracts incoming messages received by the onboard UAV router, inteprets the command, and sends it to the autopilot

## Available air-to-ground (A2G) data:

Format: Description (how data appears in sms): units

Regular Payload data:
* Arm status (Arm): 1 or 0
* Airspeed (AS): m/s
* Gndspeed (GS): m/s
* Throttle (thr): Scaled between 0.0 and 1.0
* Altitude relative to home (alt): m
* Lat (lat)
* Lon (lon)
* Status of AWS Data Telemetry Node (AWS): 1 = alive, 0 = dead
* Waypoint that has been reached (wp): Integer
* Whether aircraft is in VTOL mode (VTOL): 1 = yes, 0 = no

On-demand payload data:
* Mode (mode): String
* Status Text Messages (msg): String
* Vibration levels (vibe): m/s/s
* Clipping Events (clipping)

## Available ground-to-air (G2A) commands:

* Arm the aircraft: "arm"
* Disarm the aircraft: "disarm"
* Mode change: "mode (flight mode)"
* Set a specific waypoint number in an already-loaded mission file: "wp set (wp number)"
* Load a waypoint file that is stored in the companion computer: "wp load (absolute path to waypoint file)"
* Regular SMS update functions:
    * Activate regular SMS updates: "sms true"
    * Deactivate regular SMS updates: "sms false"
    * Set long time intervals between each update: "sms long" (on bootup, interval is long by default)
    * Set short time intervals between each update: "sms short"
* On-demand SMS update functions:
    * Request one SMS from the aircraft: "ping"
    * Request flight mode: "ping mode"
    * Request aircraft status text messages: "ping msg"
    * Request vibration levels in x, y and z axes: "ping vibe"
    * Request total number of clipping events in x, y and z axes: "ping clipping"
    * Request for statustext messages to be sent to GCS: "statustext on"
    * Request for statustext messages to not be sent to GCS: "statustext off"
    

Commands are not case sensitive

## Additional Notes

* When setting up this package, change the following in the "CMakelists.txt" file (note: instructions are taken from the following [guide](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)):

Uncomment the following line (To tell Catkin to pay attention during the catkin_make process to the setup.py file):

```
catkin_python_setup()
```

List the nodes in the install() function (To tells Catkin to install these executable files into an accessible location in the Catkin development workspace):

```
install(PROGRAMS
   scripts/SMS_tx.py
   scripts/SMS_rx.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

After the above edits are done, cd back to the bonesms_ws workspace and run `catkin_make` again

* Make sure that a text file containing whitelisted phone numbers (with the title `whitelist.txt`) is located in `~/Yonah_ROS_packages/bonesms_ws/src/air_sms/scripts/` directory
* When specifying the phone numbers, be sure to include the country code of the number
* Example file, containing one Singapore and one Malaysian phone number (`whitelist.txt`):

```
+6512345678
+60123456789
```