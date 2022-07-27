#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 

echo "Launching Cam, please wait!"

until ssh pi@10.42.0.2 "roslaunch akrobat_cam Akrobat_Master_Mapping.launch "
do 
    echo "Retry"
    sleep 2
done
 