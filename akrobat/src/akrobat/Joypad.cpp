
#include <akrobat/Akrobat.h>


using namespace std;


/** Call the joy values back.
*
*   @param mov a pointer to the constant joy message data.
*
*   @return Void.
*/
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
		case RESET:
			gaitToString = "reset";
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
			else if(mv == "reset") {
				gait = RESET;
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
				case RESET:
				{
					cout << "[  GAIT   ]: Reset" << endl;
					
					Akrobat::moveLeg(-20.0, 10.0, -90, LEFT_FRONT);
					Akrobat::moveLeg(20.0, 10.0, -90, RIGHT_FRONT);
					Akrobat::moveLeg(0.0, 10.0, -90, LEFT_MIDDLE);
					Akrobat::moveLeg(0.0, 10.0, -90, RIGHT_MIDDLE);
					Akrobat::moveLeg(20.0, 10.0, -90, LEFT_REAR);
					Akrobat::moveLeg(-20.0, 10.0, -90, RIGHT_REAR);
					
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
