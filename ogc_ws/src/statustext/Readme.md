# StatusText

## Nodes

There are two nodes, one for the air side and another for ground side. They can both be found in the `scripts/` directory. 

## Working

These nodes serve to simplify and filter the StatusText messages comming from mavros for use in Yonah's RQT GUI. 

### Air Node

The air statustext node gets data from mavros, filters and shortens it and sends it to the GCS through one of the 3 links. 
It subcribes to the `/mavros/statustext/recv/` topic and publishes to `/ogc/statustext/`

### Ground Node

The ground statustext node serves as a translater to convert the message into a readable string for display on the GUI. 
It subscribes to `/ogc/from_despatcher/statustext/` and publishes to `/ogc/yonahtext/`

### Yonah Statustext Msg

This is a custom ROS msg that contains the condensed version of the statustext. It is located in the `msg` directory.

# License
This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
