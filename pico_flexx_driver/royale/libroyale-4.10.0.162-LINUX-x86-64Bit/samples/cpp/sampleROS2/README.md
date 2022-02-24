sampleROS2
===========

Description
---------------

This tool is a interface of Royale in ROS2.

It contains the following features:
- provide a panel of rviz as the UI
- support to switch use cases
- support to set exposure time and auto exposure
- display the FPS
- provide filter function to determine the displayed distance range
- publish ROS topics: `camera_info`, `point_cloud`, `depth_image` and `gray_image`

ROS2 topics:
- `camera_info` : provide camera information
- `point_cloud` : PointCloud2 of ROS with 3 channels (x, y and z of Royale DepthData)
- `depth_image` : TYPE_32FC1 image.
- `gray_image`  : MONO16 image. The brightness of gray image can be adjusted by `gray_divisor`.


Note
----

Treating the depth data as an array of (data->width * data->height) z coordinates will lose the lens calibration.
The 3D depth points are not arranged in a rectilinear projection, and the discarded x and y coordinates from the
depth points account for the optical projection (or optical distortion) of the camera.
This means that you will see a distortion in the `depth_image`, but not in the `point_cloud`.


Dependencies
------------

- Ubuntu 18.04 (Bionic), Ubuntu 20.04 (Focal) or higher
- ROS2 Foxy Fitzroy (for Ubuntu Focal), ROS2 Eloquent Elusor (for Ubuntu Bionic)
- Royale SDK for LINUX or OSX


Installation steps for ROS2 Foxy on Ubuntu 20.04
--------------------------------------------------
1. Set GPG key with apt:
   `$ sudo apt update && sudo apt install curl gnupg2 lsb-release´
   `$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg´

2. Add ROS2 apt repository to source list:
   `$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null´

3. Install ROS2 packages
    `$ sudo apt update´
    `$ sudo apt install ros-foxy-desktop´

    For more information: Visit https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

    Installation guide for ROS2 Eloquent can be found here https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html


Steps to run the sample
------------------------
1. Create a workspace:
	`$ mkdir -p ~/ros_ws/src´

2. Download the Royale SDK for LINUX and extract it

3. Copy `sampleROS2´ from `<path_of_extracted_royale_sdk>/libroyale-<version_number>-LINUX-64Bit/samples/cpp/´
    to `~/ros2_ws/src/´

4. Install the udev rules provided by the SDK
	`$ cd <path_of_extracted_royale_sdk>´
	`$ sudo cp libroyale-<version_number>-LINUX-64Bit/driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d/´

5. Source the setup.bash file in ROS2 foxy installation
    `$ source /opt/ros/foxy/setup.bash´

6. Build the example
    `$ cd ~/ros2_ws/´
    `$ colcon build --packages-select royale_in_ros2 --cmake-args "-DCMAKE_PREFIX_PATH=/<path_of_extracted_royale_sdk>/libroyale-<version_number>-LINUX-64Bit/share"´

7. Connect camera device

8. Source the setup.bash file
	`$ source install/setup.bash´

9. Run ROS2
	`$ ros2 launch royale_in_ros2 camera_driver.launch.py´

10. Open a new Terminal to start rviz2 (3D visualization tool for ROS)
    (Note: Sometimes `$ source ~/ros2_ws/install/setup.bash` is also needed to be set in new Terminal before running the rviz2
     to ensure that the new panel of rviz2 below can be added smoothly.)
	`$ ros2 run rviz2 rviz2´

11. Add the topics in rviz2
  - point_cloud topic:
	a. Set the `Fixed frame` to `RoyaleInRos_link`;
	b. Add `PointCloud2` from `By display type`, set `Topic` to `royale_camera/point_cloud` and set `Channel Name` to `z`;
	   Or add `PointCloud2` from `By topic` and set `Channel Name` to `z`.
  - image topics:
	a. Add `Image` from `By display type` and set `Image Topic`;
	   Or add topics from `By topic`;
	b. The both image topics support sub-topic `Camera` as well.
	   Before adding these two sub-topics, the `Fixed frame` has to been set earlier.

12. Add the panel (UI) to control the camera in rviz2:
  - The menu of rviz2 --> `Panels` --> `Add New Panel` --> `royale_in_ros2` --> `RoyaleControl` --> `OK`.
  - The precise value of slider can be entered directly in the text editor and confirmed with the Enter key.
  - In auto exposure mode, the exposure time can not be manually changed.
