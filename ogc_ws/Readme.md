# Ops Ground Control Workspace (ogc_ws)

ROS Workspace containing the packages used for Ops Ground Control

## Packages

* **despatcher**: Contains ground and air despatcher nodes. Air despatcher serves as the relay between mavros and the three links on the aircraft. Ground despatcher serves as the relay between the RQt GUI and the three links on the Ground Control Station (GCS)
* **rqt**: Contains nodes to launch RQt GUI on the GCS
* **sbd**: Contains nodes to handle Iridium Short-Burst-Data (SBD) telemetry link
* **sms**: Contains nodes to handle SMS telemetry link
* **telegram**: Contains nodes to handle Telegram Cellular Data telemetry link

## License

This workspace is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details