#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/ubuntu/catkin_ws/devel/setup.bash 

echo "Launching Robot, please wait!"
while ! timeout 0.2 ping -c 1 -n 10.42.0.2 &> /dev/null
do
    printf "%c" "."
done


until roslaunch akrobat AkrobatMaster.launch

do 
    echo "Shutdown"
    ssh pi@10.42.0.2 "sudo shutdown now"
    sleep 5
    sudo shutdown now
done

