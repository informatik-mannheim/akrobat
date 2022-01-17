# Akrobat

Control and visualization of a six-legged walking robot
based on ROS.

## How to make the robo walk in a simulator with a joystick

#### Prerequisites

- Working installation of ros-noetic (follow the installation instructions on [the official ROS website](http://wiki.ros.org/noetic/Installation/Ubuntu) )

- Add your user to the dialout group (you need this because the Akrobat is controlled via the USB interface)

```sudo adduser <youruser> dialout``` 
- Packages must be installed:
- ```sudo apt-get install ros-noetic-dynamixel-sdk```
- ```sudo apt-get install ros-noetic-joy```
 

#### Steps to get the simulation running

1. Get the repo into your catkin_ws

- ```cd ~/catkin_ws/src/```

- ```git clone -b <yourbranchname> https://github.com/informatik-mannheim/akrobat.git```

2. catkin_make your workspace

- ```cd ~/catkin_ws/```

- ```catkin_make```

3. roslaunch the project with your desired running options

Launch Akrobat:
```roslaunch akrobat AkrobatMaster.launch```

Launch Akrobat gui and the simulation as well:
```roslaunch akrobat AkrobatMaster.launch gui:=true sim:=true```

Troubleshooting:

If you get an error like "[...] in the folder [akrobat] couldn't be found a file [AkrobatMaster.launch] [...]" execute this line:

```echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
