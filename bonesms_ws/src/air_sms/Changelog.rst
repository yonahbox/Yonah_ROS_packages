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