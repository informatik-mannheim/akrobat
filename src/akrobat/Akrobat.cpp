/** @file Akrobat.cpp
 *  @brief Akrobat file with main methods.
 *
 *  @author Author
 */

#include <akrobat/Akrobat.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#
#include <ros/ros.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h" 
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <akrobat/Trajectory.h>
#include <akrobat/movement.h>
#include <akrobat/Joint_position.h>




using namespace std;
using namespace tf;
using namespace ros;
using namespace angles;

/** The Akrobat constructor.
*   Initializes the leg settings for all six legs.
*   Initializes the trajectory for the different walking modes.
*   Subscribes to sensor_msgs:JointState and to akrobat::movement topics.
*/
Akrobat::Akrobat() :
	mode(0),
	gait(-1),
	rotBody(0),
	rollOver(0),

	LENGTH_COXA(72),
	LENGTH_FEMUR(92),
	LENGTH_TIBIA(162),
	scaleFacTrans(50),
	scaleFacRot(10)
{
	//float rollOv, rotOfCoxa, bdConstX, bdConstY, bdConstZ, jointInitA, jointInitB, jointInitC, minCoxa, minFemur, minTibia, maxCoxa, maxFemur, maxTibia
	legSettings[LEFT_FRONT] = LegSetting(	0.0,	180.0,	-51.0,	217.0,	0.0,	160.0,	10.0,	-90.0,	-37.5,	-99.0,	-135.0,	50.0, 96.0, 135.0);
	legSettings[RIGHT_FRONT] = LegSetting(	0.0,	0.0,	51.0,	217.0,	0.0,	20.0,	10.0,	-90.0,	-50.0,	-99.0,	-135.0,	37.5, 96.0, 135.0);
	legSettings[LEFT_MIDDLE] = LegSetting(	0.0,	180.0,	-51.0,	0.0,	0.0,	180.0,	10.0,	-90.0,	-50.0,	-99.0,	-135.0,	50.0, 96.0, 135.0);
	legSettings[RIGHT_MIDDLE] = LegSetting(	0.0,	0.0,	51.0,	0.0,	0.0,	0.0,	10.0,	-90.0,	-50.0,	-99.0,	-135.0,	50.0, 96.0, 135.0);
	legSettings[LEFT_REAR] = LegSetting(	0.0,	180.0,	-51.0,	-217.0,	0.0,	-160.0,	10.0,	-90.0,	-50.0,	-99.0,	-135.0,	50.0, 96.0, 135.0);
	legSettings[RIGHT_REAR] = LegSetting(	0.0,	0.0,	51.0,	-217.0,	0.0,	-20.0,	10.0,	-90.0,	-50.0,	-99.0,	-135.0, 50.0, 96.0, 135.0);



	//Trajectory Settings (ampWidth, ampHigh, numTick)
	trajectorySettings[TRIPOD] = TrajectorySettings(40, 40, 15);
	trajectorySettings[WAVE] = TrajectorySettings(40, 40, 15);
	trajectorySettings[RIPPLE] = TrajectorySettings(40, 40, 15);

	jointPub = n.advertise<sensor_msgs::JointState>("/goal_joint_states", 1);
	
	subMov = n.subscribe<akrobat::movement>("movements", 5, &Akrobat::callRumblePad2Back, this);
	subMov = n.subscribe<std_msgs::Bool>("shutdown", 1, &Akrobat::shutdownAkrobat, this);
	shutdownDyn = n.advertise<std_msgs::Bool>("/shutdown_dyn",1);

	
	
	jointState.name.resize(18);
	jointState.position.resize(18);
	jointState.velocity.resize(18);
	jointState.effort.resize(18);

	

	for (int i = 0, j = 1, k = 1; i < 18; i++)
	{
		string name = "m";
		name += ('0' + j);
		name += ('0' + k++);


		
		
		jointState.name[i] = name;
		
		jointState.velocity[i] = 0.0;
		jointState.effort[i] = 0.0;
		
		if (k > 3)
		{
			j++;
			k = 1;
		}
	}
	
}
/** Startup from Akrobat. To push it from the bottom.
*
*   @return Void.
*/
void Akrobat::startAkrobat()
{	
	ROS_INFO("Starting Akrobat");

	Akrobat::moveLeg(0.0, 0.0, 20, 0);
	Akrobat::moveLeg(0.0, 0.0, 20, 1);
	Akrobat::moveLeg(0.0, 0.0, 20, 2);
	Akrobat::moveLeg(0.0, 0.0, 20, 3);
	Akrobat::moveLeg(0.0, 0.0, 20, 4);
	Akrobat::moveLeg(0.0, 0.0, 20, 5);

	jointPub.publish(jointState);

	ros::Duration(3).sleep();

	Akrobat::moveLeg(0.0, -80.0, 120, 0);
	Akrobat::moveLeg(0.0, -80.0, 120, 1);
	Akrobat::moveLeg(0.0, -80.0, 120, 2);
	Akrobat::moveLeg(0.0, -80.0, 120, 3);
	Akrobat::moveLeg(0.0, -80.0, 120, 4);
	Akrobat::moveLeg(0.0, -80.0, 120, 5);

	jointPub.publish(jointState);

	ros::Duration(3).sleep();
	
	for(int l = 0; l<3 ;l++)
	{	
		int m = 5-l;
		Akrobat::moveLeg(0.0, 50.0, -120, l);
		Akrobat::moveLeg(0.0, 50.0, -120, m);

		jointPub.publish(jointState);

		ros::Duration(5.5).sleep();

	}
	

}


/** Initialize Shutdown from Akrobat.
*
*   @return Void.
*/
void Akrobat::shutdownAkrobat(const std_msgs::Bool::ConstPtr& Shutdown)
{
	bool shutdown = Shutdown->data;

	if(shutdown)
	{
		ROS_ERROR("Shutdown Now");
		for(int l = 0; l<6 ;l++)
		{
			Akrobat::moveLeg(0.0, 50.0, -120, l);
		}
		jointPub.publish(jointState);
		ros::Duration(3).sleep();

		for(int l = 0; l<6 ;l++)
		{
			Akrobat::moveLeg(0.0, 50.0, 120, l);
			ros::Duration(1).sleep();
			Akrobat::moveLeg(0.0, -80.0, 120, l);
			jointPub.publish(jointState);

			ros::Duration(5.5).sleep();

		}
		ros::Duration(3).sleep();

		for(int l = 0; l<6 ;l++)
		{
			Akrobat::moveLeg(0.0, 0.0, 20, l);
		}
		jointPub.publish(jointState);
		
		shutdownDyn.data = true;
				
		shutdownDyn.publish(shutdownDyn);


	}
}




/** Initialize the leg position of for each leg.
*
*   @return Void.
*/
void Akrobat::initAkrobat()
{	
	
	for (int legNum = 0; legNum < numberOfLegs; legNum++)
	{
		// ROS_INFO("%d", legNum);
		Transform iT; // [TRANSFORMATION DATA TYP] -- create a transform

		// [LCS] -- definition of leg coordinate system
		iT = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(LENGTH_TIBIA, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS(Vector3(0, 0, -legSettings[legNum].jointInitC), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(LENGTH_FEMUR, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS(Vector3(0, 0, -legSettings[legNum].jointInitB), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(LENGTH_COXA, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS(Vector3(-90, 0, legSettings[legNum].jointInitA), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		// [BCS] -- definition of body coordinate system
		iT = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(legSettings[legNum].bdConstX, legSettings[legNum].bdConstY, legSettings[legNum].bdConstZ));
		BodyCoordinateSystem.leg[legNum].footGlobPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		// [MainCoordinateSystem] -- definition of main coordinate system
		iT = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(0, 0, 0));
		MainCoordinateSystem.leg[legNum].footGlobPos = iT * BodyCoordinateSystem.leg[legNum].footGlobPos;

		// 3D - RVIZ init position
		jointState.header.stamp = ros::Time::now();
		Akrobat::coordinateTransformation(legNum);
		Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
		Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
		
		jointPub.publish(jointState);
		
		
	}
	
}

/** Transform source coordinate system to target coordinate system.
*
*   @param rot rotational element.
*   @param trans translational element.
*
*   @return calculated transformation for two frames.
*/
Transform Akrobat::transformCS(Vector3 rot, Vector3 trans)
{
	Transform TCS_local;
	Vector3 transVec(trans.x(), trans.y(), trans.z()); // [TRANSLATION] -- create and define vector
	TCS_local.setOrigin(transVec); // [.setOrigin] -- set translational element of transform
	Quaternion rotQuat;
	rotQuat.setRPY(from_degrees(rot.x()), from_degrees(rot.y()), from_degrees(rot.z())); // [.setRPY] -- define quaternion
	TCS_local.setRotation(rotQuat); // [.setRotaion] -- set rotational element of transform

	return TCS_local;
}



/** Transform the coordinate system for one specific leg.
*
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::coordinateTransformation(int legNum)
{
	Transform T; // [TRANSFORMATION DATA TYP] -- create a transform

	T = Akrobat::transformCS(Vector3(0, 0, 0), LegCoordinateSystem.leg[legNum].footInitPos);

	LegCoordinateSystem.leg[legNum].footPresPos = T * FootCoordinateSystem.leg[legNum].trajectoryPresPos;

	T = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(legSettings[legNum].bdConstX, legSettings[legNum].bdConstY, legSettings[legNum].bdConstZ));

	BodyCoordinateSystem.leg[legNum].footGlobPos = T * LegCoordinateSystem.leg[legNum].footPresPos;

	T = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(0, 0, 0));
	MainCoordinateSystem.leg[legNum].footGlobPos = T * BodyCoordinateSystem.leg[legNum].footGlobPos;
shutdownDyn
	T = Akrobat::transformCS(Vector3((pad.bdR.x() + rotBody), pad.bdR.y(), pad.bdR.z()), Vector3(pad.bdT.x(), (pad.bdT.y() + legSettings[legNum].rollOv), pad.bdT.z())).inverse();
	BodyCoordinateSystem.leg[legNum].footGlobPos = T * MainCoordinateSystem.leg[legNum].footGlobPos;

	T = Akrobat::transformCS(Vector3(0, 0, 0), Vector3(legSettings[legNum].bdConstX, legSettings[legNum].bdConstY, legSettings[legNum].bdConstZ)).inverse();
	LegCoordinateSystem.leg[legNum].footPresPos = T * BodyCoordinateSystem.leg[legNum].footGlobPos;

	T = Akrobat::transformCS(Vector3(0, 0, legSettings[legNum].rotOfCoxa), Vector3(0, 0, 0));
	LegCoordinateSystem.leg[legNum].footPresPos = T * LegCoordinateSystem.leg[legNum].footPresPos;

}

/** Calculate the inverse kinematics for a specific leg.
*
*   @param x the x coordinate for the current position of the leg.
*   @param y the y coordinate for the current position of the leg.
*   @param z the z coordinate for the current position of the leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::inverseKinematics(double x, double y, double z, int legNum)
{
	float R, L, ALPHA, BETA1, BETA2, BETA, GAMMA;
	
	R = sqrt(pow(y, 2) + pow(x, 2));
	ALPHA = atan2(y, x);
	L = sqrt(pow(R - LENGTH_COXA, 2) + pow(z, 2));
	BETA1 = atan2(z, (R - LENGTH_COXA));
	BETA2 = acos(0.5 * (pow(LENGTH_FEMUR, 2) + pow(L, 2) - pow(LENGTH_TIBIA, 2)) / (L * LENGTH_FEMUR));

	GAMMA = acos(0.5 * (pow(LENGTH_TIBIA, 2) + pow(LENGTH_FEMUR, 2) - pow(L, 2)) / (LENGTH_FEMUR * LENGTH_TIBIA));
	if (!rollOver)
	{
		BETA = BETA1 + BETA2;
		GAMMA = GAMMA - from_degrees(180);
	}
	else
	{
		BETA = BETA1 - BETA2;
		GAMMA = from_degrees(180) - GAMMA;
	}
	LegCoordinateSystem.leg[legNum].jointAngles.alpha = to_degrees(ALPHA); // [COXA ANGLE ] -- alpha
	LegCoordinateSystem.leg[legNum].jointAngles.beta = to_degrees(BETA); // [FEMUR ANGLE] -- beta
	LegCoordinateSystem.leg[legNum].jointAngles.gamma = to_degrees(GAMMA); // [TIBIA ANGLE] -- gamma

}

/** Move leg to target position
*
*   @param alpha Angle for Coxa.
*   @param beta Angle for Femur.
*   @param gamma Angle for Tibia.
*   @param legNum execute the operation for this specific leg.
*
*   @return 0 if not possible because one of the angles exceeds its range. 1 if successful.
*/
int Akrobat::moveLeg(float alpha, float beta, float gamma, int legNum)
{
	if (!IsWithinLimits(alpha, legSettings[legNum].minCoxa, legSettings[legNum].maxCoxa))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of coxa("
			<< legSettings[legNum].minCoxa << ":" << legSettings[legNum].maxCoxa << ") joint is exceeded " << alpha << endl;
		return 0;
	}

	if (!IsWithinLimits(beta, legSettings[legNum].minFemur, legSettings[legNum].maxFemur))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of femur("
			<< legSettings[legNum].minFemur << ":" << legSettings[legNum].maxFemur << ") joint is exceeded " << beta << endl;
		return 0;
	}

	if (!IsWithinLimits(gamma, legSettings[legNum].minTibia, legSettings[legNum].maxTibia))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of tibia("
			<< legSettings[legNum].minTibia << ":" << legSettings[legNum].maxTibia << ") joint is exceeded " << gamma << endl;
		return 0;
	}

	jointState.position[legNum * 3 + 0] = from_degrees(alpha);
	jointState.position[legNum * 3 + 1] = from_degrees(beta);
	jointState.position[legNum * 3 + 2] = from_degrees(gamma);

	return 1;
}



/** Checks if the servo motor is within his movement limits.
*
*   @param value represents the current value of the servo motor.
*   @param min represents the minimum angle of the servo motor.
*   @param max represents the maximum angle of the servo motor.
*
*   @return true or false.
*/
bool Akrobat::IsWithinLimits(const float& value, const float& min, const float& max)
{
	return value >= min && value <= max;
}

/** Checks if the pad is moved.
*
*   @return true or false.
*/
bool Akrobat::IsMoving() const
{
	return (pad.speed.x() > 0.3) || (pad.speed.x() < -0.3) || (pad.speed.y() > 0.3) || (pad.speed.y() < -0.3) || (pad.speed.z() > 0.3) || (pad.speed.z() < -0.3);
}

/** Checks if the pad is translating.
*
*   @return true or false.
*/
bool Akrobat::IsTranslating() const
{
	return (pad.bdT.x() > 0.3) || (pad.bdT.x() < -0.3) || (pad.bdT.y() > 0.3) || (pad.bdT.y() < -0.3) || (pad.bdT.z() > 0.3) || (pad.bdT.z() < -0.3);
}

/** Checks if the pad is rotating.
*
*   @return true or false.
*/
bool Akrobat::IsRotating() const
{
	return (pad.bdR.x() > 0.3) || (pad.bdR.x() < -0.3) || (pad.bdR.y() > 0.3) || (pad.bdR.y() < -0.3) || (pad.bdR.z() > 0.3) || (pad.bdR.z() < -0.3);
}
