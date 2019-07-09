#!/bin/bash

roslaunch mavros apm_test.launch
rosrun mavros gcs_bridge _gcs_url:='udp://@localhost:5001'
