#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamixel_msgs/JointState.h>

namespace Akrobat
{
	class HardwareInterface : public hardware_interface::RobotHW
	{
	public:
		HardwareInterface();

		ros::Time getTime() const { return ros::Time::now(); }
		ros::Duration getPeriod() const { return ros::Duration(0.01); }
		void readAsync(int motorArrayPosition, const dynamixel_msgs::JointStateConstPtr& motorState);
		//void read();
		void write();
	private:
		static const int numberOfMotors = 18;

		ros::NodeHandle n;

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;

		double command[numberOfMotors];
		double position[numberOfMotors];
		double velocity[numberOfMotors];
		double effort[numberOfMotors];

		ros::Publisher publisher[numberOfMotors];
		ros::Subscriber subscriber[numberOfMotors];
	};
}

#endif // HARDWAREINTERFACE_H