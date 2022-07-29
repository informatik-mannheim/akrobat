#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 

echo "Launching Robot, please wait!"

until roslaunch akrobat AkrobatMaster.launch

do 
    echo "Shutdown"
    ssh pi@10.42.0.2 "sudo shutdown now"
    sleep 5
    sudo shutdown now
done
