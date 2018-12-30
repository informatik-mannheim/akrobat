#include <akrobat/Akrobat.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <akrobat/Trajectory.h>
#include <akrobat/movement.h>

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
	legSettings[LEFT_FRONT] = LegSetting(0.0, -160.0, -51.0, 217.0, 0.0, 160.0, 10.0, -90.0, -26.0, -99.0, -135.0, 65.0, 96.0, 135.0);
	legSettings[RIGHT_FRONT] = LegSetting(0.0, -20.0, 51.0, 217.0, 0.0, 20.0, 10.0, -90.0, -71.0, -99.0, -135.0, 28.0, 96.0, 135.0);
	legSettings[LEFT_MIDDLE] = LegSetting(0.0, 180.0, -51.0, 0.0, 0.0, 180.0, 10.0, -90.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0);
	legSettings[RIGHT_MIDDLE] = LegSetting(0.0, 0.0, 51.0, 0.0, 0.0, 0.0, 10.0, -90.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0);
	legSettings[LEFT_REAR] = LegSetting(0.0, 160.0, -51.0, -217.0, 0.0, -160.0, 10.0, -90.0, -71.0, -99.0, -135.0, 30.0, 96.0, 135.0);
	legSettings[RIGHT_REAR] = LegSetting(0.0, 20.0, 51.0, -217.0, 0.0, -20.0, 10.0, -90.0, -23.0, -107.0, -135.0, 75.0, 96.0, 135.0);

	trajectorySettings[TRIPOD] = TrajectorySettings(40, 40, 15);
	trajectorySettings[WAVE] = TrajectorySettings(40, 40, 15);
	trajectorySettings[RIPPLE] = TrajectorySettings(40, 40, 15);


	jointPub = n.advertise<sensor_msgs::JointState>("/goal_joint_states", 1);

	subMov = n.subscribe<akrobat::movement>("movements", 5, &Akrobat::callRumblePad2Back, this);

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


/** Execute the gait functions respective to the current gait mode of the hexapod.
*
*   @return Void.
*/
void Akrobat::runAkrobat()
{
	if (IsMoving() || IsTranslating() || IsRotating())
	{
		jointState.header.stamp = ros::Time::now();


		for (int legNum = 0; legNum < numberOfLegs; legNum++)
		{
			switch (gait)
			{
				case TRIPOD:
				{
					Akrobat::tripodGait(&traData, legNum);
					break;
				}
				case WAVE:
				{
					Akrobat::waveGait(&traData, legNum);
					break;
				}
				case RIPPLE:
				{
					Akrobat::rippleGait(&traData, legNum);
					break;
				}
				default:
					break;
			}

			Akrobat::coordinateTransformation(legNum);
			Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
			Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
		}
	}

	jointPub.publish(jointState);
}


/** Create a tripod gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::tripodGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				if ((*tS).tick >= trajectorySettings[TRIPOD].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- down/backward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] - 2.0 * (*tS).ampX[legNum] * (*tS).tick / trajectorySettings[TRIPOD].numTick);
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] - 2.0 * (*tS).ampY[legNum] * (*tS).tick / trajectorySettings[TRIPOD].numTick);
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[TRIPOD].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[TRIPOD].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
}

/** Create a wave gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::waveGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- down/backward (1 segment of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * trajectorySettings[WAVE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * trajectorySettings[WAVE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 3;
				break;

			case 3: // [LEG MOVING] -- down/backward (2 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 4;
				break;

			case 4: // [LEG MOVING] -- down/backward (3 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 5;
				break;

			case 5: // [LEG MOVING] -- down/backward (4 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 6;
				break;

			case 6: // [LEG MOVING] -- down/backward (5 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[WAVE].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
}

/** Create a ripple gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::rippleGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		// [MOVING] --one of joypad sticks was actived
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward first half stride
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- up/forward second half stride
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 3;
				break;

			case 3: // [LEG MOVING] -- down/backward (1 segment of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 4;
				break;

			case 4: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 5;
				break;

			case 5: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 6;
				break;

			case 6: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[RIPPLE].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
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

/*********************************************************************************************************
* Function---:  Akrobat::moveLeg()
*
* Input------:	-float alpha: 	coxa joint angle
*              -float gamma: 	femur joint angle
*              -float beta:  	tibia joint angle
*              -int legNum:	execute function operation for this leg
*
* Output-----:	 0: failed
*               1: successful
*
* Overview---:	 move the leg to target position
*
* Console-Out:  F8DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
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

/*********************************************************************************************************
* Function---:  Akrobat::callRumblePad2Back()
*
* Input------:	-const sensor_msgs::Joy::ConstPtr& joy:	a pointer to const joy message data
*
* Output-----:	 None.
*
* Overview---:	 call the joy values back
*
* Console-Out:  F10DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::callRumblePad2Back(const akrobat::movement::ConstPtr& mov)
{
	std::string gaitToString = "";
	switch(gait)
	{
		case TRIPOD:
			gaitToString = "tripod";
			break;
		case RIPPLE:
			gaitToString = "ripple";
			break;
		case WAVE:
			gaitToString = "wave";
			break;
		default:
			break;
	}

	if (mov->macro == "shutdown")
	{
		cout << "SHUTTING DOWN!" << endl;
		ros::shutdown();
	}
	else
	{
		if (mov->macro == "start")
		{
			jointState.header.stamp = ros::Time::now();

			if (rollOver == 0)
			{
				rollOver = 1;
				rotBody = 180;
				legSettings[LEFT_FRONT].rotOfCoxa = 160; // rotation of leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rotOfCoxa = 20; // rotation of leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rotOfCoxa = -160; // rotation of leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rotOfCoxa = -20; // rotation of leg coordinate system (Leg 6)
				legSettings[LEFT_FRONT].rollOv = 2 * legSettings[LEFT_FRONT].bdConstY; // translation offset for leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rollOv = 2 * legSettings[RIGHT_FRONT].bdConstY;// translation offset for leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rollOv = 0; // translation offset for leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rollOv = 0; // translation offset for leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rollOv = 2 * legSettings[LEFT_REAR].bdConstY; // translation offset for leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rollOv = 2 * legSettings[RIGHT_REAR].bdConstY; // translation offset for leg coordinate system (Leg 6)

				for (int legNum = 0; legNum < numberOfLegs; legNum++)
				{
					Akrobat::coordinateTransformation(legNum);
					Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
					Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
				}
			}
			else if (rollOver == 1)
			{
				rollOver = 0;
				rotBody = 0;
				legSettings[LEFT_FRONT].rotOfCoxa = -160; // rotation of leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rotOfCoxa = -20; // rotation of leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rotOfCoxa = 160; // rotation of leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rotOfCoxa = 20; // rotation of leg coordinate system (Leg 6)
				legSettings[LEFT_FRONT].rollOv = 0; // translation offset for leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rollOv = 0; // translation offset for leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rollOv = 0; // translation offset for leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rollOv = 0; // translation offset for leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rollOv = 0; // translation offset for leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rollOv = 0; // translation offset for leg coordinate system (Leg 6)

				for (int legNum = 0; legNum < numberOfLegs; legNum++)
				{
					Akrobat::coordinateTransformation(legNum);
					Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
					Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
				}
			}

			jointPub.publish(jointState);
		}
		else if (mov->walking_mode != gaitToString)
		{
			std:string mv = mov->walking_mode;
			if(mv == "tripod")
			{				
				gait = TRIPOD;
			}			
			else if(mv == "wave")
			{
				gait = WAVE;
			}			
			else if(mv == "ripple")
			{
				gait = RIPPLE;
			}

			switch (gait)
			{
				case TRIPOD:
				{
					cout << "[  GAIT   ]: Tripod" << endl;
					traData.caseStep[LEFT_FRONT] = 2;
					traData.caseStep[RIGHT_FRONT] = 1;
					traData.caseStep[LEFT_MIDDLE] = 1;
					traData.caseStep[RIGHT_MIDDLE] = 2;
					traData.caseStep[LEFT_REAR] = 2;
					traData.caseStep[RIGHT_REAR] = 1;
					traData.initAmpX = trajectorySettings[TRIPOD].ampWidth; // [mm] X amplitude width of leg trajectory for tripod gait
					traData.initAmpY = trajectorySettings[TRIPOD].ampWidth; // [mm] Y amplitude width of leg trajectory for tripod gait
					traData.initAmpZ = trajectorySettings[TRIPOD].ampHight; // [mm] Z amplitude hight of leg trajectory for tripod gait
					traData.tick = 0;
					break;
				}

				case WAVE:
				{
					cout << "[  GAIT   ]: Wave" << endl;
					traData.caseStep[LEFT_FRONT] = 1; // .....
					traData.caseStep[RIGHT_FRONT] = 4;
					traData.caseStep[LEFT_MIDDLE] = 2;
					traData.caseStep[RIGHT_MIDDLE] = 5;
					traData.caseStep[LEFT_REAR] = 3;
					traData.caseStep[RIGHT_REAR] = 6;
					traData.initAmpX = trajectorySettings[WAVE].ampWidth;
					traData.initAmpY = trajectorySettings[WAVE].ampWidth;
					traData.initAmpZ = trajectorySettings[WAVE].ampHight;
					traData.tick = 0;
					break;
				}

				case RIPPLE:
				{
					cout << "[  GAIT   ]: Ripple" << endl;
					traData.caseStep[LEFT_FRONT] = 5; // [LEG MOVING] -- up/forward first half stride
					traData.caseStep[RIGHT_FRONT] = 2; // [LEG MOVING] -- up/forward second half stride
					traData.caseStep[LEFT_MIDDLE] = 3; // [LEG MOVING] -- down/backward (1 segment of 4)
					traData.caseStep[RIGHT_MIDDLE] = 6;
					traData.caseStep[LEFT_REAR] = 1;
					traData.caseStep[RIGHT_REAR] = 4;
					traData.initAmpX = trajectorySettings[RIPPLE].ampWidth;
					traData.initAmpY = trajectorySettings[RIPPLE].ampWidth;
					traData.initAmpZ = trajectorySettings[RIPPLE].ampHight;
					traData.tick = 0;
					break;
				}

				default:
					break;
			}
		}

			//Translation
			pad.bdT.setX((mov->commands[7] * scaleFacTrans) / 32767); // body translation X
			pad.bdT.setY((mov->commands[6] * scaleFacTrans) / 32767); // body translation Y
			pad.bdT.setZ((mov->commands[8] * scaleFacTrans) / 32767); // body translation Z
			//Walking
			if(mov->commands[1] != 0)
			{
				pad.speed.setX(-(mov->commands[1] / abs(mov->commands[1])));
			}
			else
			{
				pad.speed.setX(0);
			}
			if(mov->commands[0] != 0)
			{
				pad.speed.setY(-(mov->commands[0] / abs(mov->commands[0])));
			}
			else
			{
				pad.speed.setY(0);
			}
			if(mov->commands[2] != 0)
			{
				pad.speed.setZ(-(mov->commands[2] / abs(mov->commands[2])));
			}
			else
			{
				pad.speed.setZ(0);
			}



			//Rotation
			pad.bdR.setX((mov->commands[4] * scaleFacRot) / 32767); // body rotation X
			pad.bdR.setY((mov->commands[5] * scaleFacRot) / 32767); // body rotation Y
			pad.bdR.setZ((mov->commands[3] * scaleFacRot) / 32767); // body rotation Z
		

			if (abs(pad.speed.x()) < abs(pad.speed.y()))
			{
				// LEG 1 amplitude					     // LEG 2 amplitude
				traData.ampX[LEFT_FRONT] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_FRONT] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.y();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.y();
				// LEG 3 amplitude					     // LEG 4 amplitude
				traData.ampX[LEFT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.y();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.y();
				// LEG 5 amplitude				      	     // LEG 6 amplitude
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.y();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.y();
			}
			else if (abs(pad.speed.x()) > abs(pad.speed.y()))
			{
				// LEG 1 amplitude					     // LEG 2 amplitude
				traData.ampX[LEFT_FRONT] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_FRONT] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.x();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.x();
				// LEG 3 amplitude					     // LEG 4 amplitude
				traData.ampX[LEFT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.x();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.x();
				// LEG 5 amplitude				      	     // LEG 6 amplitude
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampY[LEFT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.x();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.x();
			}
			else if ((pad.speed.x() == 0) && (pad.speed.y() == 0) && ((pad.speed.z() >= 0) || (pad.speed.z() <= 0)))
			{
				// LEG 1 amplitude					      // LEG 2 amplitude
				traData.ampX[LEFT_FRONT] = -traData.initAmpX * pad.speed.z();
				traData.ampX[RIGHT_FRONT] = -traData.initAmpX * pad.speed.z();
				traData.ampY[LEFT_FRONT] = -traData.initAmpY * pad.speed.z();
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.z();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.z();
				// LEG 3 amplitude					      // LEG 4 amplitude
				traData.ampX[LEFT_MIDDLE] = 0;
				traData.ampX[RIGHT_MIDDLE] = 0;
				traData.ampY[LEFT_MIDDLE] = -traData.initAmpY * pad.speed.z();
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.z();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.z();
				// LEG 5 amplitude				      	     // LEG 6 amplitude
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.z();
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.z();
				traData.ampY[LEFT_REAR] = -traData.initAmpY * pad.speed.z();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.z();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.z();
		}
	}
}

bool Akrobat::IsWithinLimits(const float& value, const float& min, const float& max)
{
	return value >= min && value <= max;
}

bool Akrobat::IsMoving() const
{
	return (pad.speed.x() > 0.3) || (pad.speed.x() < -0.3) || (pad.speed.y() > 0.3) || (pad.speed.y() < -0.3) || (pad.speed.z() > 0.3) || (pad.speed.z() < -0.3);
}

bool Akrobat::IsTranslating() const
{
	return (pad.bdT.x() > 0.3) || (pad.bdT.x() < -0.3) || (pad.bdT.y() > 0.3) || (pad.bdT.y() < -0.3) || (pad.bdT.z() > 0.3) || (pad.bdT.z() < -0.3);
}

bool Akrobat::IsRotating() const
{
	return (pad.bdR.x() > 0.3) || (pad.bdR.x() < -0.3) || (pad.bdR.y() > 0.3) || (pad.bdR.y() < -0.3) || (pad.bdR.z() > 0.3) || (pad.bdR.z() < -0.3);
}
