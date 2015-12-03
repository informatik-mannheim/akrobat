#ifndef AKROBAT_H
#define AKROBAT_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <akrobat/Trajectory.h>
#include <akrobat/RumblePad2.h>
#include <akrobat/CoordinateSystem.h>
#include <akrobat/LegSetting.h>

/// The Akrobat class
/**
 * \class Akrobat
 *
 * \brief The Akrobat class is used to represent the akrobat hexapod.
 *
 * The Akrobat class is used to represent the akrobat hexapod.
 *
 */
class Akrobat
{
public:
	int mode; // [   MODE   ] -- normal(0)/translation(1)/rotation(2)
	int gait; // [   gait   ] -- tripod(1)/wave(2)/ripple(3)
	int rotBody; // [ rotBody  ] -- angle of body rotation (0/180)
	int rollOver; // [ rollOver ] -- if body roll over (0/1)

	int tripodAmpWidth;
	int tripodAmpHight;
	int tNumTick;

	int waveAmpWidth;
	int waveAmpHight;
	int wNumTick;

	int rippleAmpWidth;
	int rippleAmpHight;
	int rNumTick;

	int F1DEBUG;
	int F2DEBUG;
	int F3DEBUG;
	int F4DEBUG;
	int F5DEBUG;
	int F6DEBUG;
	int F7DEBUG;
	int F8DEBUG;
	int F9DEBUG;
	int F10DEBUG;

	int LEFT_FRONT;
	int RIGHT_FRONT;
	int LEFT_MIDDLE;
	int RIGHT_MIDDLE;
	int LEFT_REAR;
	int RIGHT_REAR;


	int LENGTH_COXA;
	int LENGTH_FEMUR;
	int LENGTH_TIBIA;

	//sticks
	int LR_stick_left;
	int UD_stick_left;
	int LR_stick_right;
	int UD_stick_right;
	//cross
	int LR_cross_key;
	int UD_cross_key;

	// TODO check joy device
	// mode cannot be detected // mode turns left stick into digital input mode
	// D-X switch can be identified with number of axis available

	int X_BUTTON;
	int A_BUTTON;
	int B_BUTTON;
	int Y_BUTTON;
	int LB_BUTTON;
	int RB_BUTTON;
	int LT_BUTTON;
	int RT_BUTTON;
	int BACK_BUTTON;
	int START_BUTTON;
	int L3_BUTTON;
	int R3_BUTTON;

	//joystick sticks scale factor
	int scaleFacTrans;
	int scaleFacRot;

	LegSetting legSettings[numberOfLegs];
	RumblePad2 pad; // [	PAD   ] -- joypad object
	Trajectory traData; // [	.. ] --
	CoordinateSystem MainCoordinateSystem, BodyCoordinateSystem, LegCoordinateSystem, FootCoordinateSystem; // [	MainCoordinateSystem...] -- coordinate system objects
	sensor_msgs::JointState js;


	// constructor
	Akrobat();

	// initialize akrobat leg position
	void initAkrobat();

	// execute important fuction to run the hexapod
	void runAkrobat();

	// create tripod gait
	void tripodGait(Trajectory* tS, int legNum);

	// create wave gait
	void waveGait(Trajectory* tS, int legNum);

	// create ripple gait
	void rippleGait(Trajectory* tS, int legNum);

	// transformate the coordinate systems
	void coordinateTransformation(int legNum);

	// calculate the inverse kinematics for each leg
	void inverseKinematics(double x, double y, double z, int legNum);

	// move the leg to target position
	int moveLeg(float alpha, float beta, float gamma, int legNum);

	// transformate source coordinate system to target coordinate system
	tf::Transform transformCS(tf::Vector3 rot, tf::Vector3 trans);

	static bool IsWithinLimits(const float& value, const float& min, const float& max);

	bool IsMoving() const;

	bool IsTranslating() const;

	bool IsRotating() const;

	// call the motor state list back
	void callRumblePad2Back(const sensor_msgs::Joy::ConstPtr& joy);

	void Debug(int i, std::string message = "") const;

private:
	ros::NodeHandle n;
	ros::Subscriber subJoy; // subscriber of joy topic
	ros::Publisher jointPub; // publisher (rviz)
};

#endif