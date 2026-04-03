# Mowgli specific ROS services

In addition to open_mower's ROS functionality there is one more ROS service available in Mowgli

    rosservice call /mowgli/ChargeCtrlSrv 28.0

It allows to modify the maximum charge voltage within the range from 25.2 to 29.4 V. During the night or other times, when the mower should pause, you may set the maximum charge voltage to a moderate voltage like 28.0 V, that does not stress the battery, if it is permanently applied. During automatic mowing, you may increase the voltage in order to obain longer mowing times.

## Requirements on Raspberry

Apply the patch 009-charge-control found in this directory to you copy of open_mower_ros

    cd $OPEN_MOWER_ROS
    patch -p1 < $MOWLI_DOCS_DIR/009-charge-control
    catkin_make

before compiling it together with the required ROS messages on the Raspberry.