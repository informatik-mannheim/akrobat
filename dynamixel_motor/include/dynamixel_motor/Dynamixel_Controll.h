#ifndef Dynamixel_Controll_H
#define Dynamixel_Controll_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <utility>      // std::pair
#include <iostream>     // std::cout

#include <stdlib.h>
#include <stdio.h>

#include <math.h>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;
using namespace ros;

typedef struct
{
  int32_t id;
  int32_t move;
  int32_t delay;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_status;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states;


  // ROS Service Client

  // Dynamixel Workbench Parameters

  string goalNodeName;

  std::map<std::string, uint32_t> dynamixel_;
  
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  
  std::pair<std::string, ItemValue> motor_info;
  sensor_msgs::JointState joint_state_msg_;
  
  int m;
  int id;
  int ID;
  int move;
  int delay;
  float pos_rad;
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
  uint8_t* goal_pos;
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[2];
  uint8_t param_move_speed[2];
  uint8_t param_delay_time[2];
  uint16_t dxl1_present_position = 0, dxl2_present_position = 0;              // Present position

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  //dynamixel::GroupSyncWrite groupSyncWrite;
  

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;

 public:
  
  
  bool get_info();
  bool motor_initialize();
  bool controler_initialize();
  bool get_positions();
  void position(const sensor_msgs::JointState::ConstPtr& msg);

 
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
