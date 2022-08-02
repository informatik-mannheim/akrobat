#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 

echo "Launching Robot, please wait!"
while ! timeout 0.2 ping -c 1 -n 10.42.0.1 &> /dev/null
do
    printf "%c" "."
done


gnome-terminal -e 'sh -c "~/catkin_ws/src/akrobat/startup/launch_master; exec bash"'


