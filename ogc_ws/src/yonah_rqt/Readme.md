# RQt Plugin

RQt is a software framework of ROS that implements the various GUI tools in the form of plugins. It is based on the Qt framework (https://www.qt.io/) and it enables users to create a UI interface that could communicate with ROS.

## Getting Started

### Prerequisites

You would need ROS and RQt installed before running the plugin. To install RQt, follow instructions from the official ROS page http://wiki.ros.org/rqt/UserGuide/Install/Groovy or if you are running a ROS kinetic on Ubuntu 16.04, follow the steps below:

```
$ sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins
$ sudo apt-get update
$ sudo apt-get dist-upgrade
```

### Installing

Once you finish installing RQt, clone the repository and follow the steps below:

1. Go to the directory where ogc_ws is located. In the example it is assumed that the repository is located at home directory. If this is not the case, just navigate accordingly
```
$ cd ~/Yonah_ROS_packages/ogc_ws/
```

2. Do a catkin_make and source the setup.bash
```
$ catkin_make
$ source ~/Yonah_ROS_packages/ogc_ws/devel/setup.bash
```

3. Run rqt and under plugin you should be able to see Yonah plugin
```
$ rqt
```
Alternatively, you can directly run the rqt plugin without needing to access the main rqt page using
```
rqt --standalone yonah_rqt
```

## Built With

* [ROS](https://www.ros.org/)
* [RQt](http://wiki.ros.org/rqt) 

