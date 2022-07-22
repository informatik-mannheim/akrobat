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
#include <akrobat/TrajectorySettings.h>
#include <akrobat/movement.h>
#include <akrobat/Joint_position.h>
#include <akrobat/Settings.h>
#include <std_msgs/Bool.h> 



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
	enum JoyPadButton : int
	{
		X = 0,
		A = 1,
		B = 2,
		Y = 3,
		LB = 4,
		RB = 5,
		LT = 6,
		RT = 7,
		BACK = 8,
		START = 9,
		L3 = 10,
		R3 = 11
	};

	enum JoyPadAxis : int
	{
		leftStickRightLeft = 0,
		leftStickUpDown = 1,
		rightStickRightLeft = 2,
		rightStickUpDown = 3,
		crossRightLeft = 4,
		crossUpDown = 5
	};

	enum LegNumber : int
	{
		LEFT_FRONT = 0,
		RIGHT_FRONT = 1,
		LEFT_MIDDLE = 2,
		RIGHT_MIDDLE = 3,
		LEFT_REAR = 4,
		RIGHT_REAR = 5
	};

	enum WalkingMode : int
	{
		TRIPOD = 0,
		WAVE = 1,
		RIPPLE = 2,
		RESET = 3
	};

	static const int numberOfWalkingPattern = 3;

	int mode; // [   MODE   ] -- normal(0)/translation(1)/rotation(2)
	int gait; // [   gait   ] -- tripod(1)/wave(2)/ripple(3)
	int rotBody; // [ rotBody  ] -- angle of body rotation (0/180)
	int rollOver; // [ rollOver ] -position- if body roll over (0/1)

	int LENGTH_COXA;
	int LENGTH_FEMUR;
	int LENGTH_TIBIA;

	// TODO check joy device
	// mode cannot be detected // mode turns left stick into digital input mode
	// D-X switch can be identified with number of axis available

	//joystick sticks scale factor
	int scaleFacTrans;
	int scaleFacRot;

	LegSetting legSettings[numberOfLegs];
	RumblePad2 pad; // [	PAD   ] -- joypad object
	Trajectory traData; // [	.. ] --
	TrajectorySettings trajectorySettings[numberOfWalkingPattern]; // 3 = number of walking modes
	CoordinateSystem MainCoordinateSystem, BodyCoordinateSystem, LegCoordinateSystem, FootCoordinateSystem; // [	MainCoordinateSystem...] -- coordinate system objects
	sensor_msgs::JointState jointState;
	std_msgs::Bool shutdownDyn;
	
	

	// constructor
	Akrobat();


	// Startup Akrobat from bottom
	void startAkrobat();

	void shutdownAkrobat(const std_msgs::Bool::ConstPtr& Shutdown);
	
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
	void callRumblePad2Back(const akrobat::movement::ConstPtr& mov);

	float alpha;
	double c;shutdown
	float twist_movment_x;
	float twist_movment_y;
	float twist_movment_z;

	void twist_mov(double Leg_x,double Leg_y,float twist,float amp, double speed, float &twist_movment_x, float &twist_movment_y, float &twist_movment_z);
	

private:
	ros::NodeHandle n;
	ros::Subscriber subMov; // subscriber of joy topic
	ros::Publisher jointPub; // publisher (rviz)
	ros::Publisher shutdownDyn;
	
};

#endif
