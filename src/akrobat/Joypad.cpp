
#include <akrobat/Akrobat.h>

#define PI 3.14159265
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
		cout << "SHUTTING DOWN! in 5 sek" << endl;
		sleep(5);
		ros::shutdown();
	}
	
	else
	{
		if (mov->macro == "roll")
		{
			jointState.header.stamp = ros::Time::now();

			if (rollOver == 0)
			{
				rollOver = 1;
				rotBody = 180;
				legSettings[LEFT_FRONT].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 6)
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
				legSettings[LEFT_FRONT].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 1)
				legSettings[RIGHT_FRONT].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 2)
				legSettings[LEFT_MIDDLE].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 3)
				legSettings[RIGHT_MIDDLE].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 4)
				legSettings[LEFT_REAR].rotOfCoxa = 180; // rotation of leg coordinate system (Leg 5)
				legSettings[RIGHT_REAR].rotOfCoxa = 0; // rotation of leg coordinate system (Leg 6)
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
			

			float a;
			a = 32767;
			//Walking
			if(mov->commands[1] != 0)
			{
				
				pad.speed.setX((mov->commands[1] / a));	
			}
			else
			{
				pad.speed.setX(0);
			}
			if(mov->commands[0] != 0)
			{
				pad.speed.setY((mov->commands[0] / a));
			}
			else
			{
				pad.speed.setY(0);
			}
			if(mov->commands[2] != 0)
			{
				
				pad.speed.setZ(-(mov->commands[2] / a));
			}
			else
			{
				pad.speed.setZ(0);
			}
		
			//Verdrehung um Z-Achse in Grad
			float maxtwist = 10*(PI/180);
			float twist;
			if(mov->commands[2] != 0)
			{
				
				twist=(mov->commands[2] / a)*maxtwist;
				//cout<<twist*(180/PI)<<endl;
			}



			//Rotation
			pad.bdR.setX((mov->commands[4] * scaleFacRot) / 32767); // body rotation X
			pad.bdR.setY((mov->commands[5] * scaleFacRot) / 32767); // body rotation Y
			pad.bdR.setZ((mov->commands[3] * scaleFacRot) / 32767); // body rotation Z


			float speed_z;
			float speed_y;
			float richtung;
			float drehung;

			if (abs(pad.speed.x()) < abs(pad.speed.y())&& ((pad.speed.z() == 0) || (pad.speed.z() == 0)))
			{	
				
				// LEG 1 amplitude					    
				traData.ampX[LEFT_FRONT] = traData.initAmpX * pad.speed.x();			
				traData.ampY[LEFT_FRONT] = traData.initAmpY * pad.speed.y();			
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.y();			
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.y();

				// LEG 3 amplitude					     
				traData.ampX[LEFT_MIDDLE] = traData.initAmpX * pad.speed.x();			
				traData.ampY[LEFT_MIDDLE] = traData.initAmpY * pad.speed.y();			
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.y();
				
				// LEG 4 amplitude
				traData.ampX[RIGHT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.y();

				// LEG 5 amplitude				      	     
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.x();				
				traData.ampY[LEFT_REAR] = traData.initAmpY * pad.speed.y();				
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.y();
				
				// LEG 6 amplitude
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.y();
			}
			else if (abs(pad.speed.x()) > abs(pad.speed.y()))
			{	
				
				// LEG 1 amplitude					     RIGHT_REAR
				traData.ampX[LEFT_FRONT] = traData.initAmpX * pad.speed.x();				
				traData.ampY[LEFT_FRONT] = traData.initAmpY * pad.speed.y();				
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.x();
				
				// LEG 2 amplitude
				traData.ampX[RIGHT_FRONT] = traData.initAmpX * pad.speed.x();
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.x();

				// LEG 3 amplitude					     
				traData.ampX[LEFT_MIDDLE] = traData.initAmpX * pad.speed.x();				
				traData.ampY[LEFT_MIDDLE] = traData.initAmpY * pad.speed.y();				
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.x();				

				// LEG 4 amplitude
				traData.ampX[RIGHT_MIDDLE] = traData.initAmpX * pad.speed.x();
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.x();

				// LEG 5 amplitude				      	     
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.x();			
				traData.ampY[LEFT_REAR] = traData.initAmpY * pad.speed.y();			
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.x();
				
				// LEG 6 amplitude
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.x();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.y();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.x();
			}
			else if ((pad.speed.x() == 0) && (pad.speed.y() == 0) && ((pad.speed.z() >= 0) || (pad.speed.z() <= 0)))
			{
				// LEG 0 amplitude					      
				traData.ampX[LEFT_FRONT] = -traData.initAmpX * pad.speed.z();				
				traData.ampY[LEFT_FRONT] = -traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ * pad.speed.z();	
				
				// LEG 1 amplitude
				traData.ampX[RIGHT_FRONT] = -traData.initAmpX * pad.speed.z();
				traData.ampY[RIGHT_FRONT] = traData.initAmpY * pad.speed.z();
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ * pad.speed.z();

				// LEG 2 amplitude					      
				traData.ampX[LEFT_MIDDLE] = 0;
				traData.ampY[LEFT_MIDDLE] = -traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ * pad.speed.z();				

				// LEG 3 amplitude
				traData.ampX[RIGHT_MIDDLE] = 0;
				traData.ampY[RIGHT_MIDDLE] = traData.initAmpY * pad.speed.z();
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ * pad.speed.z();

				// LEG 4 amplitude				      	     
				traData.ampX[LEFT_REAR] = traData.initAmpX * pad.speed.z();
				traData.ampY[LEFT_REAR] = -traData.initAmpY * pad.speed.z();
				traData.ampZ[LEFT_REAR] = traData.initAmpZ * pad.speed.z();				

				// LEG 5 amplitude
				traData.ampX[RIGHT_REAR] = traData.initAmpX * pad.speed.z();
				traData.ampY[RIGHT_REAR] = traData.initAmpY * pad.speed.z();
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ * pad.speed.z();
			}

		
		//Drehung in Bewegung
			else if (abs(pad.speed.x()) < abs(pad.speed.y()) && (pad.speed.z() > 0 || pad.speed.z() < 0) )
				{	
				//cout<<"y: "<< pad.speed.y()<<" z: "<<pad.speed.z()<<endl;
				speed_z = abs(pad.speed.z());
				speed_y = abs(pad.speed.y());
				richtung = pad.speed.y()/speed_y;
				drehung = pad.speed.z()/speed_z;
				// LEG 0 amplitude
				traData.ampX[LEFT_FRONT] = -(traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[LEFT_FRONT] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[LEFT_FRONT] = traData.initAmpZ *2 * (speed_y);
				
				// LEG 1 amplitude
				//Akrobat::twist_mov(legSettings[RIGHT_FRONT].bdConstX,legSettings[RIGHT_FRONT].bdConstY,twist,traData.initAmpX,pad.speed.y(),twist_movment_x,twist_movment_y,twist_movment_z);
				traData.ampX[RIGHT_FRONT] = -(traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[RIGHT_FRONT] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[RIGHT_FRONT] = traData.initAmpZ*2 * (speed_y);


				// LEG 2 amplitude
				//Akrobat::twist_mov(legSettings[LEFT_MIDDLE].bdConstX,legSettings[LEFT_MIDDLE].bdConstY,twist,traData.initAmpX,pad.speed.y(),twist_movment_x,twist_movment_y,twist_movment_z);					      
				traData.ampX[LEFT_MIDDLE] = -(traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[LEFT_MIDDLE] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[LEFT_MIDDLE] = traData.initAmpZ*2 * (speed_y);							

				// LEG 3 amplitude
				//Akrobat::twist_mov(legSettings[RIGHT_MIDDLE].bdConstX,legSettings[RIGHT_MIDDLE].bdConstY,twist,traData.initAmpX,pad.speed.y(),twist_movment_x,twist_movment_y,twist_movment_z);
				traData.ampX[RIGHT_MIDDLE] = (traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[RIGHT_MIDDLE] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[RIGHT_MIDDLE] = traData.initAmpZ*2 * (speed_y);

				// LEG 4 amplitude
				//Akrobat::twist_mov(legSettings[LEFT_REAR].bdConstX,legSettings[LEFT_REAR].bdConstY,twist,traData.initAmpX,pad.speed.y(),twist_movment_x,twist_movment_y,twist_movment_z);				      	     
				traData.ampX[LEFT_REAR] = (traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[LEFT_REAR] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[LEFT_REAR] = traData.initAmpZ*2 * (speed_y);				

				// LEG 5 amplitude
				//Akrobat::twist_mov(legSettings[RIGHT_REAR].bdConstX,legSettings[RIGHT_REAR].bdConstY,twist,traData.initAmpX,pad.speed.y(),twist_movment_x,twist_movment_y,twist_movment_z);
				traData.ampX[RIGHT_REAR] = (traData.initAmpX/2) * (speed_y+speed_z)*richtung*drehung;
				traData.ampY[RIGHT_REAR] = (traData.initAmpY/2) * (speed_y+speed_z)*richtung;
				traData.ampZ[RIGHT_REAR] = traData.initAmpZ*2 * (speed_y);
				}		
		
		
	}
}

void Akrobat::twist_mov(double Leg_x,double Leg_y,float twist,float amp, double speed, float &twist_movment_x, float &twist_movment_y, float &twist_movment_z)
{
float x_n;
float y_n;

alpha = abs(atan(Leg_y/Leg_x));
c = sqrt((Leg_y*Leg_y)+(Leg_x*Leg_x));
cout << "alpha: " << alpha << " c: " << c <<endl;

//Leg 0
if (Leg_x < 0 && Leg_y >0)
{

	x_n = Leg_x + cos(alpha+twist)*c;
	y_n = sin(alpha+twist)*c - Leg_y;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n )* speed)/4;
	twist_movment_z = amp * speed;
}
//Leg1
else if (Leg_x > 0 && Leg_y > 0 )
{

	x_n=cos(alpha-twist)*c - Leg_x;
	y_n=sin(alpha-twist)*c - Leg_y;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n)* speed)/4;
	twist_movment_z = amp * speed;
}
//Leg2
else if (Leg_x < 0 && Leg_y == 0 )
{

	x_n=Leg_x + cos(alpha+twist)*c ;
	y_n=sin(alpha+twist)*c - Leg_y;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n)* speed)/4;
	twist_movment_z = amp * speed;
}
//Leg3
else if (Leg_x > 0 && Leg_y == 0 )
{

	x_n= cos(alpha-twist)*c - Leg_x;
	y_n=sin(alpha-twist)*c - Leg_y;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n)* speed)/4;
	twist_movment_z = amp * speed;
}
//Leg 4
if (Leg_x < 0 && Leg_y < 0)
{
	cout << Leg_x <<endl;
	
	x_n = Leg_x + cos(alpha-twist)*c ;
	y_n = Leg_y + sin(alpha-twist)*c;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n )* speed)/4;
	twist_movment_z = amp * speed;
}
//Leg5
else if (Leg_x > 0 && Leg_y < 0 )
{

	x_n=cos(alpha+twist)*c - Leg_x;
	y_n=sin(alpha+twist)*c + Leg_y;

	cout << "x_N: "<< x_n << " y_n: " << y_n << endl;

	twist_movment_x = ((amp/cos(twist)+ x_n)* speed)/4;
	twist_movment_y = ((amp/sin(twist)+ y_n)* speed)/4;
	twist_movment_z = amp * speed;
}
}
