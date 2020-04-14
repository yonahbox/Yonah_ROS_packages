# SMS (Ops)

ROS Package to handle Yonah's SMS Telemetry for Operational Usage. This package is designed to be used in tandem with the despatcher package

## Nodes

**sms_link**, located in `src` folder

## Modules

**RutOS**, located in `scripts` folder

## Installation notes

After creating the package, copy the `CMakelists.txt` and `package.xml` files in this folder to your sms package, before running `catkin_make` again in the `ogc_ws` workspace

Change the `client_phone_no` parameter in the launch files (located in the `launch` folder) to the phone number of the receipient. For example, if this package is deployed on the aircraft, specify the phone number of the GCS, and vice-versa. Be sure the include the country code of the number (e.g. +6512345678, instead of 12345678))

Make sure that a text file containing whitelisted phone numbers (with the title `whitelist.txt`) is located in `src` directory

* When specifying the phone numbers, be sure to include the country code of the number
* Example file, containing one Singapore and one Malaysian phone number (`whitelist.txt`):

```
+6512345678
+60123456789
```

## Testing

Use the launch files in the `launch` folder for testing. The SMS nodes will be launched together with either the air or ground despatcher nodes (in the despatcher package).

## License

This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details