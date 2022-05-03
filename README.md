# Akrobat

Control and visualization of a six-legged walking robot
based on ROS.

## How to make the robo walk in a simulator with a joystick

#### Prerequisites

- Working installation of ros-melodic (follow the installation instructions on [the official ROS website](http://wiki.ros.org/kinetic/Installation/Ubuntu) )

- Install the ROS Kinetic joystick 

```sudo apt-get install ros-kinetic-joy```

- Add your user to the dialout group (you need this because the Akrobat is controlled via the USB interface)

```sudo add <youruser> dialout``` 
 

#### Steps to get the simulation running

1. Get the repo into your catkin_ws

- ```cd ~/catkin_ws/src/```

- ```git clone -b <yourbranchname> github.com/informatik-mannheim/akrobat```

2. catkin_make your workspace

- ```cd ~/catkin_ws/```

- ```catkin_make```

3. roslaunch the project with your desired running options

Launch Akrobat as simulation:
```roslaunch akrobat AkrobatMaster.launch```

Launch Akrobat robot and the simulation as well:
```roslaunch akrobat AkrobatMaster.launch gui:=true```

Troubleshooting:

If you get an error like "[...] in the folder [akrobat] couldn't be found a file [AkrobatMaster.launch] [...]" execute this line:

```echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
