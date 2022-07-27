#!/usr/bin/env bash

# Launch the robot
#source /opt/ros/noetic/setup.bash 
#source /home/user/catkin_ws/devel/setup.bash 

echo "Launching Cam, please wait!"
ssh pi@10.42.0.2
until roslaunch akrobat_cam Akrobat_Master_Mapping.launch
do 
    echo "Shutdown"
    sleep 2
    #sudo shutdown now
done
 