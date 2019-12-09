# air_sms

Package for handling Yonah's SMS Telemetry

## air_sms Nodes

* **SMS_Tx**: Handles sending of messages to the onboard UAV router, which will proceed to send it to Ground Control
* **SMS_Rx**: Extracts incoming messages received by the onboard UAV router, inteprets the command, and sends it to the autopilot

## Additional Notes

* Make sure that a text file containing whitelisted phone numbers (with the title `whitelist.txt`) is located in `~/Yonah_ROS_packages/bonesms_ws/src/air_sms/scripts/` directory
* When specifying the phone numbers, be sure to include the country code of the number
* Make sure there is an empty line at the end of the text file.
* Example file, containing one Singapore and one Malaysian phone number (`whitelist.txt`):

```
+6512345678
+60123456789

```

Note the blank line at the end of the file!