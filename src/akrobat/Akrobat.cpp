#include <akrobat/Akrobat.h>

#include <cmath>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <akrobat/akrobat_init.h>
#include <akrobat/TrajectoryStruct.h>

using namespace std;
using namespace tf;
using namespace ros;
using namespace angles;

/*********************************************************************************************************
* Function---:  Akrobat::Akrobat()
*
* Input------:	 None.
*
* Output-----:	 None.
*
* Overview---:	 constructor
*
* Note-------:	 None
********************************************************************************************************/
Akrobat::Akrobat() : mode(0), gait(0), rotBody(0), rollOver(0)
{
	// TODO left off: 
	// need to put settings into settings class replaces usages
	// merge legs/configuration
	// move controller config into separate class
	// rethink overall structure

	legSettings[LEFT_FRONT] = LegSetting(0, -160, -51, 217, 0, 160, 10, -90, -26, -99, -135, 65, 135, 96);
	legSettings[RIGHT_FRONT] = LegSetting(0, -20, 51, 217, 0, 20, 10, -90, -71, -99, -135, 28, 96, 135);
	legSettings[LEFT_MIDDLE] = LegSetting(0, 180, -51, 0, 0, 180, 10, -90, -51, -99, -135, 48, 96, 135);
	legSettings[RIGHT_MIDDLE] = LegSetting(0, 0, 51, 0, 0, 0, 10, -90, -51, -99, -135, 48, 96, 135);
	legSettings[LEFT_REAR] = LegSetting(0, 160, -51, -217, 0, -160, 10, -90, -71, -99, -135, 30, 96, 135);
	legSettings[RIGHT_REAR] = LegSetting(0, 20, 51, -217, 0, -20, 10, -90, -23, -107, -135, 75, 96, 135);

	rotOfCoxa[LEFT_FRONT] = -160;
	rotOfCoxa[RIGHT_FRONT] = -20;
	rotOfCoxa[LEFT_MIDDLE] = 180;
	rotOfCoxa[RIGHT_MIDDLE] = 0;
	rotOfCoxa[LEFT_REAR] = 160;
	rotOfCoxa[RIGHT_REAR] = 20;

	// body constant initialization
	bdConstX[LEFT_FRONT] = -51;
	bdConstX[RIGHT_FRONT] = 51;
	bdConstX[LEFT_MIDDLE] = -51;
	bdConstX[RIGHT_MIDDLE] = 51;
	bdConstX[LEFT_REAR] = -51;
	bdConstX[RIGHT_REAR] = 51;

	bdConstY[LEFT_FRONT] = 217;
	bdConstY[RIGHT_FRONT] = 217;
	bdConstY[LEFT_MIDDLE] = 0;
	bdConstY[RIGHT_MIDDLE] = 0;
	bdConstY[LEFT_REAR] = -217;
	bdConstY[RIGHT_REAR] = -217;

	bdConstZ[LEFT_FRONT] = 0;
	bdConstZ[RIGHT_FRONT] = 0;
	bdConstZ[LEFT_MIDDLE] = 0;
	bdConstZ[RIGHT_MIDDLE] = 0;
	bdConstZ[LEFT_REAR] = 0;
	bdConstZ[RIGHT_REAR] = 0;

	// joint angle initialization
	jointInitA[LEFT_FRONT] = 160;
	jointInitA[RIGHT_FRONT] = 20;
	jointInitA[LEFT_MIDDLE] = 180;
	jointInitA[RIGHT_MIDDLE] = 0;
	jointInitA[LEFT_REAR] = -160;
	jointInitA[RIGHT_REAR] = -20;

	jointInitB[LEFT_FRONT] = 10;
	jointInitB[RIGHT_FRONT] = 10;
	jointInitB[LEFT_MIDDLE] = 10;
	jointInitB[RIGHT_MIDDLE] = 10;
	jointInitB[LEFT_REAR] = 10;
	jointInitB[RIGHT_REAR] = 10;


	jointInitC[LEFT_FRONT] = -90;
	jointInitC[RIGHT_FRONT] = -90;
	jointInitC[LEFT_MIDDLE] = -90;
	jointInitC[RIGHT_MIDDLE] = -90;
	jointInitC[LEFT_REAR] = -90;
	jointInitC[RIGHT_REAR] = -90;

	// min limit of coxa joint initialization
	minCoxa[LEFT_FRONT] = -26;
	minCoxa[RIGHT_FRONT] = -71;
	minCoxa[LEFT_MIDDLE] = -51;
	minCoxa[RIGHT_MIDDLE] = -51;
	minCoxa[LEFT_REAR] = -71;
	minCoxa[RIGHT_REAR] = -23;

	minFemur[LEFT_FRONT] = -99;
	minFemur[RIGHT_FRONT] = -99;
	minFemur[LEFT_MIDDLE] = -99;
	minFemur[RIGHT_MIDDLE] = -99;
	minFemur[LEFT_REAR] = -99;
	minFemur[RIGHT_REAR] = -107;

	minTibia[LEFT_FRONT] = -135;
	minTibia[RIGHT_FRONT] = -135;
	minTibia[LEFT_MIDDLE] = -135;
	minTibia[RIGHT_MIDDLE] = -135;
	minTibia[LEFT_REAR] = -135;
	minTibia[RIGHT_REAR] = -135;

	// max limit of coxa jointinitialization
	maxCoxa[LEFT_FRONT] = 65;
	maxCoxa[RIGHT_FRONT] = 28;
	maxCoxa[LEFT_MIDDLE] = 48;
	maxCoxa[RIGHT_MIDDLE] = 48;
	maxCoxa[LEFT_REAR] = 30;
	maxCoxa[RIGHT_REAR] = 75;


	maxFemur[LEFT_FRONT] = 96;
	maxFemur[RIGHT_FRONT] = 96;
	maxFemur[LEFT_MIDDLE] = 96;
	maxFemur[RIGHT_MIDDLE] = 96;
	maxFemur[LEFT_REAR] = 96;
	maxFemur[RIGHT_REAR] = 96;

	maxTibia[LEFT_FRONT] = 135;
	maxTibia[RIGHT_FRONT] = 135;
	maxTibia[LEFT_MIDDLE] = 135;
	maxTibia[RIGHT_MIDDLE] = 135;
	maxTibia[LEFT_REAR] = 135;
	maxTibia[RIGHT_REAR] = 135;

	for (int i = 0; i < numberOfLegs; i++)
	{
		rollOv[i] = 0.0f;
	}

	// [SUBCRIBER]	-- subJoy:  subscribe the topic(joy)
	// 		-- subMots: subscribe the topic(/motorState/pan_tilt_port/) test
	subJoy = n.subscribe<sensor_msgs::Joy>("joy", 10, &Akrobat::callRumblePad2Back, this);
	jointPub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

	// [PUBLISHER] -- pubLegXJointX: publish the joint angles of each leg to topic(/controller_mxx/command)
	// LEG 1
	pubLeg1Joint1 = n.advertise<std_msgs::Float64>("/m11/command", 1);
	pubLeg1Joint2 = n.advertise<std_msgs::Float64>("/m12/command", 1);
	pubLeg1Joint3 = n.advertise<std_msgs::Float64>("/m13/command", 1);
	// LEG 2
	pubLeg2Joint1 = n.advertise<std_msgs::Float64>("/m21/command", 1);
	pubLeg2Joint2 = n.advertise<std_msgs::Float64>("/m22/command", 1);
	pubLeg2Joint3 = n.advertise<std_msgs::Float64>("/m23/command", 1);
	// LEG 3
	pubLeg3Joint1 = n.advertise<std_msgs::Float64>("/m31/command", 1);
	pubLeg3Joint2 = n.advertise<std_msgs::Float64>("/m32/command", 1);
	pubLeg3Joint3 = n.advertise<std_msgs::Float64>("/m33/command", 1);
	// LEG 4
	pubLeg4Joint1 = n.advertise<std_msgs::Float64>("/m41/command", 1);
	pubLeg4Joint2 = n.advertise<std_msgs::Float64>("/m42/command", 1);
	pubLeg4Joint3 = n.advertise<std_msgs::Float64>("/m43/command", 1);
	// LEG 5
	pubLeg5Joint1 = n.advertise<std_msgs::Float64>("/m51/command", 1);
	pubLeg5Joint2 = n.advertise<std_msgs::Float64>("/m52/command", 1);
	pubLeg5Joint3 = n.advertise<std_msgs::Float64>("/m53/command", 1);
	// LEG 6
	pubLeg6Joint1 = n.advertise<std_msgs::Float64>("/m61/command", 1);
	pubLeg6Joint2 = n.advertise<std_msgs::Float64>("/m62/command", 1);
	pubLeg6Joint3 = n.advertise<std_msgs::Float64>("/m63/command", 1);

	// ARRAY SIZE
	js.name.resize(18);
	js.position.resize(18);
	js.velocity.resize(18);
	js.effort.resize(18);
}// Akrobat::Akrobat()

/*********************************************************************************************************
* Function---:  Akrobat::initAkrobat()
*
* Input------:	 None.
*
* Output-----:	 None.
*
* Overview---:	 initialize akrobat leg position
*
* Console-Out:  F1DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::initAkrobat()
{
	for (int legNum = 0; legNum < numberOfLegs; legNum++)
	{
		// ROS_INFO("%d", legNum);
		Transform iT; // [TRANSFORMATION DATA TYP] -- create a transform

		// [LCS] -- definition of leg coordinate system
		iT = Akrobat::transformCS("TIBIA", "ENDEFFCTR", Vector3(0, 0, 0), Vector3(LENGTH_TIBIA, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS("TIBIA", "TIBIA", Vector3(0, 0, -jointInitC[legNum]), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS("FEMUR", "TIBIA", Vector3(0, 0, 0), Vector3(LENGTH_FEMUR, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS("FEMUR", "FEMUR", Vector3(0, 0, -jointInitB[legNum]), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS("COXA", "FEMUR", Vector3(0, 0, 0), Vector3(LENGTH_COXA, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		iT = Akrobat::transformCS("COXA", "COXA", Vector3(-90, 0, jointInitA[legNum]), Vector3(0, 0, 0));
		LegCoordinateSystem.leg[legNum].footInitPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		// [BCS] -- definition of body coordinate system
		iT = Akrobat::transformCS("LCS", "BCS", Vector3(0, 0, 0), Vector3(bdConstX[legNum], bdConstY[legNum], bdConstZ[legNum]));
		BodyCoordinateSystem.leg[legNum].footGlobPos = iT * LegCoordinateSystem.leg[legNum].footInitPos;

		// [MainCoordinateSystem] -- definition of main coordinate system
		iT = Akrobat::transformCS("BCS", "MCS", Vector3(0, 0, 0), Vector3(0, 0, 0));
		MainCoordinateSystem.leg[legNum].footGlobPos = iT * BodyCoordinateSystem.leg[legNum].footGlobPos;

		// 3D - RVIZ init position
		js.header.stamp = ros::Time::now();
		Akrobat::coordinateTransformation(legNum);
		Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
		Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
		jointPub.publish(js);

		// [OUTPUT] -- console
#if F1DEBUG == 1 // -------->[MACRO] -- akrobat_init.h

		cout << "|----------------------------INITAKROBAT()------------------------LN: " << legNum << "|" << endl;
		cout << setw(40) << "..x.." << setw(12) << "..y.." << setw(12) << "..y.." << endl;

		// [OUTPUT] -- output of LegCoordinateSystem.leg[legNum].footInitPos vector
		cout << " LegCoordinateSystem.leg[" << legNum << "].footInitPos---: ";
		cout << setw(12) << LegCoordinateSystem.leg[legNum].footInitPos.x();
		cout << setw(12) << LegCoordinateSystem.leg[legNum].footInitPos.y();
		cout << setw(12) << LegCoordinateSystem.leg[legNum].footInitPos.z() << endl;

		// [OUTPUT] -- output of BodyCoordinateSystem.leg[legNum].footGlobPos vector
		cout << " BodyCoordinateSystem.leg[" << legNum << "].footGlobPos---: ";
		cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.x();
		cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.y();
		cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.z() << endl;

		// [OUTPUT] -- output of MainCoordinateSystem.leg[legNum].footGlobPos vector
		cout << " MainCoordinateSystem.leg[" << legNum << "].footGlobPos---: ";
		cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.x();
		cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.y();
		cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.z() << endl;
		cout << endl;
#endif
	}// FOR (legNum)

	//ros::shutdown();
}// Akrobat::initAkrobat()

/*********************************************************************************************************
* Function---:  Akrobat::runAkrobat()
*
* Input------:	 None.
*
* Output-----:	 None.
*
* Overview---:	 execute important fuction to run the hexapod
*
* Console-Out:  F2DEBUG (akrobat_init.h)
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::runAkrobat()
{
	if (MOVING || TRANSLATION || ROTATION)
	{
		// cout<<"runAkr"<<endl;
		js.header.stamp = ros::Time::now();
		for (int legNum = 0; legNum < numberOfLegs; legNum++)
		{
			if (gait == 1)
			{
				Akrobat::tripodGait(&traData, legNum);
			}
			if (gait == 2)
			{
				Akrobat::waveGait(&traData, legNum);
			}
			if (gait == 3)
			{
				Akrobat::rippleGait(&traData, legNum);
			}
			Akrobat::coordinateTransformation(legNum);
			Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
			Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
		}// FOR (legNum)
	}// MOVING
	jointPub.publish(js);
}// Akrobat::runAkrobat()

/*********************************************************************************************************
* Function---:  Akrobat::tripodGait()
*
* Input------:	-trajectoryStruct *tS: 	include the data of trajectory for each leg
*              -int legNum:            execute function operation for this leg
*
* Output-----:	 None.
*
* Overview---:	 create tripod gait
*
* Console-Out:  F3DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::tripodGait(trajectoryStruct* tS, int legNum)
{
	if (MOVING)
	{ // [MOVING] -- one of joypad sticks was actived
		switch ((*tS).caseStep[legNum])
		{
		case 1: // [LEG MOVING] -- up/forward
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / tNumTick));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / tNumTick));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / tNumTick));
			if ((*tS).tick >= tNumTick - 1) (*tS).caseStep[legNum] = 2;
			break;

		case 2: // [LEG MOVING] -- down/backward
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] - 2.0 * (*tS).ampX[legNum] * (*tS).tick / tNumTick);
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] - 2.0 * (*tS).ampY[legNum] * (*tS).tick / tNumTick);
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= tNumTick - 1) (*tS).caseStep[legNum] = 1;
			break;
		}// SWITCH ((*tS).caseStep[legNum])
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > tNumTick - 1)
			{
				(*tS).tick = 0;
			}
		}

		// [OUTPUT] -- console
#if F3DEBUG == 1 // -----> [SET MARCO] -- akrobat_init.h

		cout << "|-------------------------tripodGait()-----------------------LN: " << legNum << "|" << endl;
		cout << "LEG: " << legNum << "/ CASE: " << (*tS).caseStep[legNum] << "/ TICK: " << setw(2) << (*tS).tick << "-> ";
		cout << setw(5) << ".x." << setw(5) << ".y." << setw(5) << ".y." << endl;

		// [OUTPUT] -- output of FootCoordinateSystem.leg[legNum].trajectoryPresPos vector
		cout << "FootCoordinateSystem.leg[" << legNum << "].tcryPresPos-: ";
		cout << setw(8) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.x());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.y());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.z());// <<endl;

		// [OUTPUT] -- output of LegCoordinateSystem.leg[legNum].footPresPos vector
		cout << "LegCoordinateSystem.leg[" << legNum << "].footPresPos-: ";
		cout << setw(8) << round(LegCoordinateSystem.leg[legNum].footPresPos.x());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.y());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.z()) << endl;
		// cout<<endl;
#endif
	}// IF (MOVING)
}// Akrobat::tripodGait(trajectoryStruct *tS,int legNum)

/*********************************************************************************************************
* Function---:  Akrobat::waveGait()
*
* Input------:	-trajectoryStruct *tS: 	include the data of trajectory for each leg
*              -int legNum:            execute function operation for this leg
*
* Output-----:	 None.
*
* Overview---:	 create wave gait
*
* Console-Out:  F4DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::waveGait(trajectoryStruct* tS, int legNum)
{
	if (MOVING)
	{ // [MOVING] -- one of joypad sticks was actived
		switch ((*tS).caseStep[legNum])
		{
		case 1: // [LEG MOVING] -- up/forward
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / wNumTick));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / wNumTick));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / wNumTick));
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 2;
			break;

		case 2: // [LEG MOVING] -- down/backward (1 segment of 5)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * wNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * wNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 3;
			break;

		case 3: // [LEG MOVING] -- down/backward (2 segments of 5)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 4;
			break;

		case 4: // [LEG MOVING] -- down/backward (3 segments of 5)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 5;
			break;

		case 5: // [LEG MOVING] -- down/backward (4 segments of 5)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 6;
			break;

		case 6: // [LEG MOVING] -- down/backward (5 segments of 5)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * wNumTick) / (5.0 * wNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= wNumTick - 1) (*tS).caseStep[legNum] = 1;
			break;
		}// SWITCH ((*tS).caseStep[legNum])
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > wNumTick - 1)
			{
				(*tS).tick = 0;
			}
		}
		// [OUTPUT] -- console
#if F4DEBUG == 1 // -----> [SET MARCO] -- akrobat_init.h

		cout << "|-------------------------tripodGait()-----------------------LN: " << legNum << "|" << endl;
		cout << "LEG: " << legNum << "/ CASE: " << (*tS).caseStep[legNum] << "/ TICK: " << setw(2) << (*tS).tick << "-> ";
		cout << setw(5) << ".x." << setw(5) << ".y." << setw(5) << ".y." << endl;

		// [OUTPUT] -- output of FootCoordinateSystem.leg[legNum].trajectoryPresPos vector
		cout << "FootCoordinateSystem.leg[" << legNum << "].tcryPresPos-: ";
		cout << setw(8) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.x());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.y());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.z()) << endl;

		// [OUTPUT] -- output of LegCoordinateSystem.leg[legNum].footPresPos vector
		cout << "LegCoordinateSystem.leg[" << legNum << "].footPresPos-: ";
		cout << setw(8) << round(LegCoordinateSystem.leg[legNum].footPresPos.x());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.y());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.z()) << endl;
		cout << endl;
#endif
	}// IF (MOVING)
}// Akrobat::waveGait(trajectoryStruct *tS,int legNum)

/*********************************************************************************************************
* Function---:  Akrobat::rippleGait()
*
* Input------:	-trajectoryStruct *tS: 	include the data of trajectory for each leg
*		-int legNum:		execute function operation for this leg
*
* Output-----:	 None.
*
* Overview---:	 create ripple gait
*
* Console-Out:  F5DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::rippleGait(trajectoryStruct* tS, int legNum)
{
	if (MOVING)
	{
		// [MOVING] --one of joypad sticks was actived
		switch ((*tS).caseStep[legNum])
		{
		case 1: // [LEG MOVING] -- up/forward first half stride
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / (2.0 * rNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / (2.0 * rNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / (2.0 * rNumTick)));
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 2;
			break;

		case 2: // [LEG MOVING] -- up/forward second half stride
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * ((*tS).tick + rNumTick) / (2.0 * rNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * ((*tS).tick + rNumTick) / (2.0 * rNumTick)));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * ((*tS).tick + rNumTick) / (2.0 * rNumTick)));
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 3;
			break;

		case 3: // [LEG MOVING] -- down/backward (1 segment of 4)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 4;
			break;

		case 4: // [LEG MOVING] -- down/backward (2 segments of 4)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 5;
			break;

		case 5: // [LEG MOVING] -- down/backward (2 segments of 4)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 6;
			break;

		case 6: // [LEG MOVING] -- down/backward (2 segments of 4)
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * rNumTick) / (4.0 * rNumTick))));
			FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
			if ((*tS).tick >= rNumTick - 1) (*tS).caseStep[legNum] = 1;
			break;
		}// SWITCH ((*tS).caseStep[legNum])
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > rNumTick - 1)
			{
				(*tS).tick = 0;
			}
		}
		// [OUTPUT] -- console
#if F5DEBUG == 1 // -----> [SET MARCO] -- akrobat_init.h

		cout << "|-------------------------tripodGait()-----------------------LN: " << legNum << "|" << endl;
		cout << "LEG: " << legNum << "/ CASE: " << (*tS).caseStep[legNum] << "/ TICK: " << setw(2) << (*tS).tick << "-> ";
		cout << setw(5) << ".x." << setw(5) << ".y." << setw(5) << ".y." << endl;

		// [OUTPUT] -- output of FootCoordinateSystem.leg[legNum].trajectoryPresPos vector
		cout << "FootCoordinateSystem.leg[" << legNum << "].tcryPresPos-: ";
		cout << setw(8) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.x());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.y());
		cout << setw(5) << round(FootCoordinateSystem.leg[legNum].trajectoryPresPos.z()) << endl;

		// [OUTPUT] -- output of LegCoordinateSystem.leg[legNum].footPresPos vector
		cout << "LegCoordinateSystem.leg[" << legNum << "].footPresPos-: ";
		cout << setw(8) << round(LegCoordinateSystem.leg[legNum].footPresPos.x());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.y());
		cout << setw(5) << round(LegCoordinateSystem.leg[legNum].footPresPos.z()) << endl;
		cout << endl;
#endif
	}// IF (MOVING)
	// FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
}// Akrobat::rippleGait(int legNum)

/*********************************************************************************************************
* Function---:  Akrobat::coordinateTransformation()
*
* Input------:	-int legNum:	execute function operation for this leg
*
* Output-----:	 None.
*
* Overview---:	 transformate the coordinate systems
*
* Console-Out:  F6DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
void Akrobat::coordinateTransformation(int legNum)
{
	Transform T; // [TRANSFORMATION DATA TYP] -- create a transform

	T = Akrobat::transformCS("FCS", "LCS", Vector3(0, 0, 0), LegCoordinateSystem.leg[legNum].footInitPos);
	LegCoordinateSystem.leg[legNum].footPresPos = T * FootCoordinateSystem.leg[legNum].trajectoryPresPos;

	T = Akrobat::transformCS("LCS", "BCS", Vector3(0, 0, 0), Vector3(bdConstX[legNum], bdConstY[legNum], bdConstZ[legNum]));
	BodyCoordinateSystem.leg[legNum].footGlobPos = T * LegCoordinateSystem.leg[legNum].footPresPos;

	T = Akrobat::transformCS("BCS", "MCS", Vector3(0, 0, 0), Vector3(0, 0, 0));
	MainCoordinateSystem.leg[legNum].footGlobPos = T * BodyCoordinateSystem.leg[legNum].footGlobPos;

	T = Akrobat::transformCS("MCS", "BCS", Vector3((pad.bdR.x() + rotBody), pad.bdR.y(), pad.bdR.z()), Vector3(pad.bdT.x(), (pad.bdT.y() + rollOv[legNum]), pad.bdT.z()));
	BodyCoordinateSystem.leg[legNum].footGlobPos = T * MainCoordinateSystem.leg[legNum].footGlobPos;

	T = Akrobat::transformCS("BCS", "LCS", Vector3(0, 0, 0), Vector3(bdConstX[legNum], bdConstY[legNum], bdConstZ[legNum]));
	LegCoordinateSystem.leg[legNum].footPresPos = T * BodyCoordinateSystem.leg[legNum].footGlobPos;

	T = Akrobat::transformCS("LCS", "LCS", Vector3(0, 0, rotOfCoxa[legNum]), Vector3(0, 0, 0));
	LegCoordinateSystem.leg[legNum].footPresPos = T * LegCoordinateSystem.leg[legNum].footPresPos;

	// [OUTPUT] -- console
#if F6DEBUG == 1 // -----> [SET MARCO] -- akrobat_init.h

	cout << "|-------------------coordinateTransformation()----------------LN: " << legNum << "|" << endl;
	cout << setw(40) << "..x.." << setw(12) << "..y.." << setw(12) << "..y.." << endl;
	cout << " MainCoordinateSystem.leg[" << legNum << "].footGlobPos---: ";
	cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.x();
	cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.y();
	cout << setw(12) << MainCoordinateSystem.leg[legNum].footGlobPos.z() << endl;

	cout << " BodyCoordinateSystem.leg[" << legNum << "].footGlobPos---: ";
	cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.x();
	cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.y();
	cout << setw(12) << BodyCoordinateSystem.leg[legNum].footGlobPos.z() << endl;

	cout << " LegCoordinateSystem.leg[" << legNum << "].footPresPos---: ";
	cout << setw(12) << LegCoordinateSystem.leg[legNum].footPresPos.x();
	cout << setw(12) << LegCoordinateSystem.leg[legNum].footPresPos.y();
	cout << setw(12) << LegCoordinateSystem.leg[legNum].footPresPos.z() << endl;
	cout << endl;
#endif
}// Akrobat::coordinateTransformation(int legNum)

/*********************************************************************************************************
* Function---:  Akrobat::inverseKinematics()
*
* Input------: -int legNum:	execute function operation for this leg
*
* Output-----:	 None.
*
* Overview---:	 calculate the inverse kinematics for each leg
*
* Console-Out:  F7DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
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

	// [OUTPUT] -- console
#if F7DEBUG == 1 // -----> [SET MARCO] -- akrobat_init.h

	cout << "|----------------------inverseKinematics()------------------LN: " << legNum << "|" << endl;
	cout << setw(40) << "..x.." << setw(12) << "..y.." << setw(12) << "..y.." << endl;
	cout << setw(40) << "CAngl" << setw(12) << "FAngl" << setw(12) << "TAngl" << endl;
	cout << "LegCoordinateSystem.leg[" << legNum << "].jointAngles---: ";
	cout << "counter : " << cnt++ << "---: ";
	cout << setw(12) << LegCoordinateSystem.leg[legNum].jointAngles.alpha;
	cout << setw(12) << LegCoordinateSystem.leg[legNum].jointAngles.beta;
	cout << setw(12) << LegCoordinateSystem.leg[legNum].jointAngles.gamma;
	cout << setw(12) << "Atan" << atan2(0, 160) << endl;
#endif
}// Akrobat::inverseKinematics(int legNum)

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
	if (IsWithinLimits(LegCoordinateSystem.leg[legNum].jointAngles.alpha, minCoxa[legNum], maxCoxa[legNum]))
	{ // [COXA  JOINT LIMITS] -- control the joint max and min limits
		if (IsWithinLimits(LegCoordinateSystem.leg[legNum].jointAngles.beta, minFemur[legNum], maxFemur[legNum]))
		{ // [FEMUR JOINT LIMITS] -- control the joint max and min limits
			if (IsWithinLimits(LegCoordinateSystem.leg[legNum].jointAngles.gamma, minTibia[legNum], maxTibia[legNum]))
			{ // [TIBIA JOINT LIMITS] -- control the joint max and min limits
				std_msgs::Float64 aux; // [STANDARD MESSAGE TYPE] -- publishing data type
				// [...publish(..)] -- function to publish data
				switch (legNum)
				{
				case 0: // [LEG 1] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg1Joint1.publish(aux);
					js.name[0] = "m11";
					js.position[0] = aux.data;
					js.velocity[0] = 0.0;
					js.effort[0] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg1Joint2.publish(aux);
					js.name[1] = "m12";
					js.position[1] = aux.data;
					js.velocity[1] = 0.0;
					js.effort[1] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg1Joint3.publish(aux);
					js.name[2] = "m13";
					js.position[2] = aux.data;
					js.velocity[2] = 0.0;
					js.effort[2] = 0.0;
					break;

				case 1: // [LEG 2] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg2Joint1.publish(aux);
					js.name[3] = "m21";
					js.position[3] = aux.data;
					js.velocity[3] = 0.0;
					js.effort[3] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg2Joint2.publish(aux);
					js.name[4] = "m22";
					js.position[4] = aux.data;
					js.velocity[4] = 0.0;
					js.effort[4] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg2Joint3.publish(aux);
					js.name[5] = "m23";
					js.position[5] = aux.data;
					js.velocity[5] = 0.0;
					js.effort[5] = 0.0;
					break;

				case 2: // [LEG 3] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg3Joint1.publish(aux);
					js.name[6] = "m31";
					js.position[6] = aux.data;
					js.velocity[6] = 0.0;
					js.effort[6] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg3Joint2.publish(aux);
					js.name[7] = "m32";
					js.position[7] = aux.data;
					js.velocity[7] = 0.0;
					js.effort[7] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg3Joint3.publish(aux);
					js.name[8] = "m33";
					js.position[8] = aux.data;
					js.velocity[8] = 0.0;
					js.effort[8] = 0.0;
					break;

				case 3: // [LEG 4] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg4Joint1.publish(aux);
					js.name[9] = "m41";
					js.position[9] = aux.data;
					js.velocity[9] = 0.0;
					js.effort[9] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg4Joint2.publish(aux);
					js.name[10] = "m42";
					js.position[10] = aux.data;
					js.velocity[10] = 0.0;
					js.effort[10] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg4Joint3.publish(aux);
					js.name[11] = "m43";
					js.position[11] = aux.data;
					js.velocity[11] = 0.0;
					js.effort[11] = 0.0;
					break;

				case 4: // [LEG 5] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg5Joint1.publish(aux);
					js.name[12] = "m51";
					js.position[12] = aux.data;
					js.velocity[12] = 0.0;
					js.effort[12] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg5Joint2.publish(aux);
					js.name[13] = "m52";
					js.position[13] = aux.data;
					js.velocity[13] = 0.0;
					js.effort[13] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg5Joint3.publish(aux);
					js.name[14] = "m53";
					js.position[14] = aux.data;
					js.velocity[14] = 0.0;
					js.effort[14] = 0.0;
					break;

				case 5: // [LEG 6] -- set angle values and publish
					aux.data = from_degrees(alpha);
					pubLeg6Joint1.publish(aux);
					js.name[15] = "m61";
					js.position[15] = aux.data;
					js.velocity[15] = 0.0;
					js.effort[15] = 0.0;

					aux.data = from_degrees(beta);
					pubLeg6Joint2.publish(aux);
					js.name[16] = "m62";
					js.position[16] = aux.data;
					js.velocity[16] = 0.0;
					js.effort[16] = 0.0;

					aux.data = from_degrees(gamma);
					pubLeg6Joint3.publish(aux);
					js.name[17] = "m63";
					js.position[17] = aux.data;
					js.velocity[17] = 0.0;
					js.effort[17] = 0.0;
					break;
				}

				return 1;

				// [OUTPUT WARNING] -- range warnings of joint (limit valu's: akrobat_init.h)
			}
			else
			{
				cout << "[WARNING] " << "LEG " << legNum << ": angle range of tibia(" << minTibia[legNum] << ":" << maxTibia[legNum] << ") joint is exceeded " << LegCoordinateSystem.leg[legNum].jointAngles.gamma << endl;
			}
		}
		else
		{
			cout << "[WARNING] " << "LEG " << legNum << ": angle range of femur(" << minFemur[legNum] << ":" << maxFemur[legNum] << ") joint is exceeded " << LegCoordinateSystem.leg[legNum].jointAngles.beta << endl;
		}
	}
	else
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of coxa(" << minCoxa[legNum] << ":" << maxCoxa[legNum] << ") joint is exceeded " << LegCoordinateSystem.leg[legNum].jointAngles.alpha << endl;
	}
	return 0;
}// int Akrobat::moveLeg(float alpha, float beta, float gamma, int legNum)

/*********************************************************************************************************
* Function---:  Akrobat::transformCS()
*
* Input------:	-string sCS: 		source frame
*              -string tCS:		target frame
*              -Vector3 rot:		rotational element
*              -Vector3 trans:		translational element
*
* Output-----:	-TCS: calculated transformation for two frames
*
* Overview---:	 transformate source coordinate system to target coordinate system
*
* Console-Out:  F9DEBUG (akrobat_init.h) 1:output 0:no output
*
* Note-------:	 None.
********************************************************************************************************/
Transform Akrobat::transformCS(string sCS, string tCS, Vector3 rot, Vector3 trans)
{


	Transform TCS_local = Transform::getIdentity(); // [TRANSFORMATION DATA TYP] -- create a transform
	Vector3 transVec(trans.x(), trans.y(), trans.z()); // [TRANSLATON] -- create and define vector
	TCS_local.setOrigin(transVec); // [.setOrigin] -- set translational element of transform
	Quaternion rotQuat = Quaternion::getIdentity(); // [ROTATION] -- create quaternion
	rotQuat.setRPY(from_degrees(rot.x()), from_degrees(rot.y()), from_degrees(rot.z())); // [.setRPY] -- define quaternion
	TCS_local.setRotation(rotQuat); // [.setRotaion] -- set rotational element of transform

	if ((sCS == "LCS") && (tCS == "FCS"))
	{
		return TCS_local.inverse();
	}

	if ((sCS == "BCS") && (tCS == "LCS"))
	{
		return TCS_local.inverse();
	}

	if ((sCS == "MCS") && (tCS == "BCS"))
	{
		return TCS_local.inverse();
	}

	if ((sCS == "FCS") && ((tCS == "LCS") || (tCS == "FCS")))
	{
		return TCS_local;
	}

	if ((sCS == "LCS") && ((tCS == "BCS") || (tCS == "LCS")))
	{
		return TCS_local;
	}

	if ((sCS == "BCS") && ((tCS == "MCS") || (tCS == "BCS")))
	{
		return TCS_local;
	}

	if ((sCS == "COXA") && ((tCS == "FEMUR") || (tCS == "COXA")))
	{
		return TCS_local;
	}

	if ((sCS == "FEMUR") && ((tCS == "TIBIA") || (tCS == "FEMUR")))
	{
		return TCS_local;
	}

	if ((sCS == "TIBIA") && ((tCS == "ENDEFFCTR") || (tCS == "TIBIA")))
	{
		return TCS_local;
	}

	// [ERROR OUTPUT] -- if the sCS (source coord. system) or tCS (target coord. system) does not exist
	cout << "[ERROR]: FAILED: Transform for this frame does not exist!" << endl;
	return TCS_local;
}// Transform Akrobat::transformCS(string sCS, string tCS, Vector3 rot, Vector3 trans)

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
void Akrobat::callRumblePad2Back(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons[LB_BUTTON] && joy->buttons[RB_BUTTON])
	{
		cout << "SHUTTING DOWN!" << endl;
		ros::shutdown();
	}
	else
	{
		if (joy->buttons[RB_BUTTON] && !(joy->buttons[LB_BUTTON]))
		{
			mode = 0;
			cout << "[OPERATION]: Normal" << endl;
			gait = 0;
		}

		if (joy->buttons[RT_BUTTON] && !joy->buttons[B_BUTTON])
		{
			mode = 1;
			cout << "[OPERATION]: Translation" << endl;
			gait = 0;
		}

		if (joy->buttons[R3_BUTTON])
		{
			mode = 2;
			cout << "[OPERATION]: Rotation" << endl;
			gait = 0;
		}

		if (joy->buttons[START_BUTTON])
		{ // rollOver=1;  cout<<"rollover: "<< rollOver <<endl;
			js.header.stamp = ros::Time::now();
			if (rollOver == 0)
			{
				rollOver = 1;
				rotBody = 180;
				legSettings[LEFT_FRONT].rotOfCoxa = 160; // rotation of leg coordinate system (Leg 1)
legSettings[RIGHT_FRONT].rotOfCoxa = 20; // rotation of leg coordinate system (Leg 2)
				rotOfCoxa[LEFT_MIDDLE] = 180; // rotation of leg coordinate system (Leg 3)
				rotOfCoxa[RIGHT_MIDDLE] = 0; // rotation of leg coordinate system (Leg 4)
				rotOfCoxa[LEFT_REAR] = -160; // rotation of leg coordinate system (Leg 5)
				rotOfCoxa[RIGHT_REAR] = -20; // rotation of leg coordinate system (Leg 6)
				legSettings[LEFT_FRONT].rollOv = 2 * legSettings[LEFT_FRONT].bdConstY; // translation offset for leg coordinate system (Leg 1)
legSettings[RIGHT_FRONT].rollOv = 2 *legSettings[RIGHT_FRONT].bdConstY;// translation offset for leg coordinate system (Leg 2)
				rollOv[LEFT_MIDDLE] = 0; // translation offset for leg coordinate system (Leg 3)
				rollOv[RIGHT_MIDDLE] = 0; // translation offset for leg coordinate system (Leg 4)
				rollOv[LEFT_REAR] = 2 * bdConstY[LEFT_REAR]; // translation offset for leg coordinate system (Leg 5)
				rollOv[RIGHT_REAR] = 2 * bdConstY[RIGHT_REAR]; // translation offset for leg coordinate system (Leg 6)
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
				rotOfCoxa[LEFT_MIDDLE] = 180; // rotation of leg coordinate system (Leg 3)
				rotOfCoxa[RIGHT_MIDDLE] = 0; // rotation of leg coordinate system (Leg 4)
				rotOfCoxa[LEFT_REAR] = 160; // rotation of leg coordinate system (Leg 5)
				rotOfCoxa[RIGHT_REAR] = 20; // rotation of leg coordinate system (Leg 6)
				legSettings[LEFT_FRONT].rollOv = 0; // translation offset for leg coordinate system (Leg 1)
legSettings[RIGHT_FRONT].rollOv = 0; // translation offset for leg coordinate system (Leg 2)
				rollOv[LEFT_MIDDLE] = 0; // translation offset for leg coordinate system (Leg 3)
				rollOv[RIGHT_MIDDLE] = 0; // translation offset for leg coordinate system (Leg 4)
				rollOv[LEFT_REAR] = 0; // translation offset for leg coordinate system (Leg 5)
				rollOv[RIGHT_REAR] = 0; // translation offset for leg coordinate system (Leg 6)
				for (int legNum = 0; legNum < numberOfLegs; legNum++)
				{
					Akrobat::coordinateTransformation(legNum);
					Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
					Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
				}
			}
			jointPub.publish(js);
		}

		if (joy->buttons[LB_BUTTON] && !(joy->buttons[RB_BUTTON]) && mode == 0)
		{
			gait = 1;
			cout << "[  GAIT   ]: Tripod" << endl;
			traData.caseStep[LEFT_FRONT] = 2;
			traData.caseStep[RIGHT_FRONT] = 1;
			traData.caseStep[LEFT_MIDDLE] = 1;
			traData.caseStep[RIGHT_MIDDLE] = 2;
			traData.caseStep[LEFT_REAR] = 2;
			traData.caseStep[RIGHT_REAR] = 1;
			traData.initAmpX = tripodAmpWidth; // [mm] X amplitude width of leg trajectory for tripod gait
			traData.initAmpY = tripodAmpWidth; // [mm] Y amplitude width of leg trajectory for tripod gait
			traData.initAmpZ = tripodAmpHight; // [mm] Z amplitude hight of leg trajectory for tripod gait
			traData.tick = 0;
		}

		if (joy->buttons[LT_BUTTON] && mode == 0)
		{
			gait = 2;
			cout << "[  GAIT   ]: Wave" << endl;
			traData.caseStep[LEFT_FRONT] = 1; // .....
			traData.caseStep[RIGHT_FRONT] = 4;
			traData.caseStep[LEFT_MIDDLE] = 2;
			traData.caseStep[RIGHT_MIDDLE] = 5;
			traData.caseStep[LEFT_REAR] = 3;
			traData.caseStep[RIGHT_REAR] = 6;
			traData.initAmpX = waveAmpWidth;
			traData.initAmpY = waveAmpWidth;
			traData.initAmpZ = waveAmpHight;
			traData.tick = 0;
		}

		if (joy->buttons[L3_BUTTON] && mode == 0)
		{
			gait = 3;
			cout << "[  GAIT   ]: Ripple" << endl;
			traData.caseStep[LEFT_FRONT] = 5; // [LEG MOVING] -- up/forward first half stride
			traData.caseStep[RIGHT_FRONT] = 2; // [LEG MOVING] -- up/forward second half stride
			traData.caseStep[LEFT_MIDDLE] = 3; // [LEG MOVING] -- down/backward (1 segment of 4)
			traData.caseStep[RIGHT_MIDDLE] = 6;
			traData.caseStep[LEFT_REAR] = 1;
			traData.caseStep[RIGHT_REAR] = 4;
			traData.initAmpX = rippleAmpWidth;
			traData.initAmpY = rippleAmpWidth;
			traData.initAmpZ = rippleAmpHight;
			traData.tick = 0;
		}

		if (mode == 1)
		{// [mode 1][OPERATION] -- Translation
			pad.bdT.setX(joy->axes[LR_stick_left] * scaleFacTrans); // body translation X
			pad.bdT.setY(joy->axes[UD_stick_left] * scaleFacTrans); // body translation Y
			pad.bdT.setZ(joy->axes[UD_stick_right] * scaleFacTrans);// body translation Z
			pad.speed.setX(0);
			pad.speed.setY(0);
			pad.speed.setZ(0);
			pad.bdR.setX(0);
			pad.bdR.setY(0);
			pad.bdR.setZ(0);
		}// IF (mode 1)
		else if (mode == 2)
		{// [mode 2][OPERATION] -- Rotation
			pad.bdR.setX(joy->axes[UD_stick_left] * scaleFacRot); // body rotation X
			pad.bdR.setY(joy->axes[LR_stick_left] * scaleFacRot); // body rotation Y
			pad.bdR.setZ(joy->axes[LR_stick_right] * scaleFacRot); // body rotation Z
			pad.speed.setX(0);
			pad.speed.setY(0);
			pad.speed.setZ(0);
			pad.bdT.setX(0);
			pad.bdT.setY(0);
			pad.bdT.setZ(0);
		}// ELSEIF (mode 2)
		else
		{// [mode 0][OPERATION] -- Normal
			pad.bdT.setX(0);
			pad.bdT.setY(0);
			pad.bdT.setZ(0);
			pad.bdR.setX(0);
			pad.bdR.setY(0);
			pad.bdR.setZ(0);
			if ((joy->axes[LR_stick_left] > 0.3) || (joy->axes[LR_stick_left] < -0.3))
			{ // sideward movement
				pad.speed.setX(-(joy->axes[LR_stick_left]) / abs(joy->axes[LR_stick_left]));
			}
			else
			{
				pad.speed.setX(0);
			}

			if ((joy->axes[UD_stick_left] > 0.3) || (joy->axes[UD_stick_left] < -0.3))
			{ // forward/backward movement
				pad.speed.setY(joy->axes[UD_stick_left] / abs(joy->axes[UD_stick_left]));
			}
			else
			{
				pad.speed.setY(0);
			}

			if ((joy->axes[LR_stick_right] > 0.3) || (joy->axes[LR_stick_right] < -0.3))
			{ // rotational movement
				pad.speed.setZ((joy->axes[LR_stick_right] / abs(joy->axes[LR_stick_right])));
			}
			else
			{
				pad.speed.setZ(0);
			}

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
}

bool Akrobat::IsWithinLimits(const float& value, const float& min, const float& max)
{
	return value >= min && value <= max;
}