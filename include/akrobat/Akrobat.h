#ifndef AKROBAT_H
#define AKROBAT_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>

#include <akrobat/akrobat_init.h>

class Akrobat
{
private:
	ros::NodeHandle n;
	ros::Subscriber subJoy;          //subscriber of joy topic
	ros::Publisher jointPub;         //publisher (rviz)

	//LEG1
	ros::Publisher  pubLeg1Joint1;	//publicher for jointX of legX
	ros::Publisher  pubLeg1Joint2;	// .....
	ros::Publisher  pubLeg1Joint3;
	//LEG2
	ros::Publisher  pubLeg2Joint1;
	ros::Publisher  pubLeg2Joint2;
	ros::Publisher  pubLeg2Joint3;
	//LEG3
	ros::Publisher  pubLeg3Joint1;
	ros::Publisher  pubLeg3Joint2;
	ros::Publisher  pubLeg3Joint3;
	//LEG4
	ros::Publisher  pubLeg4Joint1;
	ros::Publisher  pubLeg4Joint2;
	ros::Publisher  pubLeg4Joint3;
	//LEG5
	ros::Publisher  pubLeg5Joint1;
	ros::Publisher  pubLeg5Joint2;
	ros::Publisher  pubLeg5Joint3;
	//LEG6
	ros::Publisher  pubLeg6Joint1;
	ros::Publisher  pubLeg6Joint2;
	ros::Publisher  pubLeg6Joint3;   // .....

public:
	//constructor
	Akrobat();

	//initialize akrobat leg position
	void initAkrobat();

	//execute important fuction to run the hexapod
	void runAkrobat();

	//create tripod gait
	void tripodGait(trajectoryStruct *tS, int legNum);

	//create wave gait
	void waveGait(trajectoryStruct *tS, int legNum);

	//create ripple gait
	void rippleGait(trajectoryStruct *tS, int legNum);

	//transformate the coordinate systems
	void coordinateTransformation(int legNum);

	//calculate the inverse kinematics for each leg
	void inverseKinematics(double x, double y, double z, int legNum);

	//move the leg to target position
	int moveLeg(float alpha, float beta, float gamma, int legNum);

	//transformate source coordinate system to target coordinate system
	tf::Transform transformCS(std::string sourceCS, std::string targetCS, tf::Vector3 rot, tf::Vector3 trans);

	//call the motor state list back
	void callRumblePad2Back(const sensor_msgs::Joy::ConstPtr& joy);
};

#endif