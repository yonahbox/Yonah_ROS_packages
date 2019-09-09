# Bonesms 

Workspace that contains *air_sms*, a package for handling SMS telemetry

This package will be used on the BeagleBone Black companion computer that is connected to the Cube autopilot

## Notes

* Make sure that a text file containing whitelisted phone numbers (with the title `whitelist.txt`) is located in `~/Yonah_ROS_packages/bonesms_ws/src/air_sms/src/` directory
* When specifying the phone numbers, be sure to include the country code of the number
* Make sure there is an empty line at the end of the text file.
* Example file, containing one Singapore and one Malaysian phone number (`whitelist.txt`):

```
+6512345678
+60123456789

```

Note the blank line at the end of the file!