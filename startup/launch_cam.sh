#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 

echo "Launching Cam, please wait!"

until ssh pi@10.42.0.2 "~/catkin_ws/src/akrobat_cam/launch_cam.sh"
do 
    echo "Retry"
    sleep 2
done
 