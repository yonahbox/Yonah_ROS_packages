# Yonah ROS Packages	

 This repository contains all custom ROS packages written for Yonah's flight/operational usage	

 ## List of folders	

* launch: Contains launch file for MAVROS, data, and SMS nodes. Take note of the following:
    * Copy the `apm_pluginlists.yaml` and `apm_config.yaml` file from the launch folder and put it into the mavros workspace: `/opt/ros/kinetic/share/mavros/launch/`
* bonesms_ws: ROS workspace containing air_sms package that handles SMS Telemetry for technical development. See `bonesms_ws/src/air_sms/Readme.md` for more information
* bonedata_ws: ROS workspace containing air_data package that handles Cellular Data Telemetry for technical development
* ogc_ws: ROS workspace containing packages pertaining to development of Ops Ground Control (OGC)