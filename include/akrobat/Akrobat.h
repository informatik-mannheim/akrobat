#ifndef AKROBAT_H
#define AKROBAT_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Joy.h>

#include <akrobat/TrajectoryStruct.h>
#include <akrobat/RumblePad2Struct.h>
#include <akrobat/CoordinateSystemStruct.h>

class Akrobat
{
public:
	int mode; // [   MODE   ] -- normal(0)/translation(1)/rotation(2)
	int gait; // [   gait   ] -- tripod(1)/wave(2)/ripple(3)
	int rotBody; // [ rotBody  ] -- angle of body rotation (0/180)
	int rollOver; // [ rollOver ] -- if body roll over (0/1)
	int ON; // [  ON   ] -- if null akrobat shutting down
	float rollOv[numberOfLegs]; // LCS translational correction after body roll over
	float rotOfCoxa[numberOfLegs]; // rotates abot coxa for angle 45° init
																	// body constant initialization
	float bdConstX[numberOfLegs]; // [mm] half hight of body
	float bdConstY[numberOfLegs]; // [mm] half width of body
	float bdConstZ[numberOfLegs]; // [mm] half length of body
														 // joint angle initialization
	float jointInitA[numberOfLegs]; // [°] (coxa joint) alpha angle init
	float jointInitB[numberOfLegs]; // [°] (femur joint) beta angle init
	float jointInitC[numberOfLegs]; // [°] (tibia joint) gamma angle init
																	   // min limit of coxa joint initialization
	float minCoxa[numberOfLegs]; // [°] (coxa joint) alpha angle min limit
	float minFemur[numberOfLegs]; // [°] (femur joint) beta angle min limit
	float minTibia[numberOfLegs]; // [°] (tibia joint) gamma angle min limit
																		   // max limit of coxa jointinitialization
	float maxCoxa[numberOfLegs]; // [°] (coxa joint) alpha angle max limit
	float maxFemur[numberOfLegs]; // [°] (femur joint) beta angle max limit
	float maxTibia[numberOfLegs]; // [°] (tibia joint) gamma angle max limit

	rumblePad2Struct pad; // [	PAD   ] -- joypad object
	trajectoryStruct traData; // [	.. ] --
	coordinateSystemStruct MCS, BCS, LCS, FCS; // [	MCS...] -- coordinate system objects
	sensor_msgs::JointState js;

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
};

#endif