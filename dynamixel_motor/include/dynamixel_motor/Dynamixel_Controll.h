#ifndef Dynamixel_Controll_H
#define Dynamixel_Controll_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dynamixel_motor/dynamixel.h>
#include "akrobat/movement.h"

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
  int32_t id;                                                             // Motor ID
  int32_t move;                                                           // Moving Speed from Motor
  int32_t delay;                                                          // Return_Delay_Time from Motor
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Topic Publisher
  

  sensor_msgs::JointState jointState;
  dynamixel_motor::dynamixel dynamixel_msg;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states;
  ros::Subscriber dyn_status;
  ros::Subscriber mov_status;


  string goalNodeName;

  std::map<std::string, uint32_t> dynamixel_;
  
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  
  std::pair<std::string, ItemValue> motor_info;                             // Motor Infos
  std::vector<std::string> motor;

  int m;

  // Parameter for ItemValue
  int id;
  int move;
  int delay;
  int ID;
  
  float pos_rad;
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;                                       // Communication result
  int dxl_read_result = COMM_TX_FAIL;     
  bool dxl_addparam_result = false;                                         // addParam result
  bool dxl_getdata_result = false;
  uint8_t* goal_pos;
  uint8_t dxl_error = 0;                                                    // Dynamixel error
  uint8_t param_goal_position[2];                                           // Goal_Position for Sync Write
  uint8_t param_move_speed[2];                                              // Move Speed for Sync Write
  uint8_t param_delay_time[2];                                              // Delay_Time for Sync Write
  uint16_t dxl1_present_position = 0, dxl2_present_position = 0;            // Present position
  uint16_t dxl1_present_load = 0;                                               // Present Load

  dynamixel::PortHandler *portHandler;

  dynamixel::PacketHandler *packetHandler;


 public:
  bool get_info();
  bool motor_initialize();
  bool controler_initialize();
  bool sub_positions();
  bool sub_status();
  void chatterCallback(const dynamixel_motor::dynamixel & msg);
  bool cur_position();
  void position(const sensor_msgs::JointState::ConstPtr& msg);
  bool sub_down();
  void torqueoff(const akrobat::movement::ConstPtr& msg);
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
