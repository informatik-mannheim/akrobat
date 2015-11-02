#ifndef AKROBAT_INIT_H
#define AKROBAT_INIT_H

//--------------------------INCLUDE
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

//-------------------------NAMESPACE
using namespace std;
using namespace tf;
using namespace ros;


//---------------------OUTPUT MACRO 
#define  F1DEBUG 	0 //initAkrobat()  
#define  F2DEBUG 	0 //runAkrobat()
#define  F3DEBUG 	0 //tripodGait() 
#define  F4DEBUG 	0 //waveGait()
#define  F5DEBUG 	0 //rippleGait()
#define  F6DEBUG 	0 //coordinateTransformation()
#define  F7DEBUG 	0 //inverseKinematics()
#define  F8DEBUG 	0 //moveLeg()
#define  F9DEBUG 	0 //transformCS()
#define  F10DEBUG 	0 //callRumblePad2Back()


//-----------------------BODY MACRO
#define halfBodyHight  	0     //[mm] half hight of body
#define halfBodyWidth  	51    //[mm] half width of body
#define halfBodyLength 	217   //[mm] half length of body


//------------------------LEG MACRO
#define LEFT_FRONT    	0 //[ID'S] COXA: 11 / FEMUR: 12 / TIBIA: 13
#define RIGHT_FRONT     1 //[ID'S] COXA: 21 / FEMUR: 22 / TIBIA: 23
#define LEFT_MIDDLE     2 //[ID'S] COXA: 31 / FEMUR: 32 / TIBIA: 33
#define RIGHT_MIDDLE	3 //[ID'S] COXA: 41 / FEMUR: 42 / TIBIA: 43
#define LEFT_REAR       4 //[ID'S] COXA: 51 / FEMUR: 52 / TIBIA: 53
#define RIGHT_REAR      5 //[ID'S] COXA: 61 / FEMUR: 62 / TIBIA: 63

#define numberOfJoints  3
#define numberOfLegs    6

#define LENGTH_COXA     72  //[mm] distance coxa - femur !!!!!!!!!!!!!!!!!!!!!!!
#define LENGTH_FEMUR    92  //[mm] distance femur - tabia !!!!!!!!!!!!!!!!!!!!!!!
#define LENGTH_TIBIA    162 //[mm] distance tabia - end effector !!!!!!!!!!!!!!!!!!!!!!!


//-------------------JOYSTICK MACRO
// definitions for F710 in "direct input mode" (switch on 
// back side in "D" position), recognized as Logitech Cordless Rumblepad 2 

//sticks 
#define LR_stick_left   0
#define UD_stick_left   1
#define LR_stick_right  2
#define UD_stick_right  3
//cross
#define LR_cross_key    4
#define UD_cross_key    5

#define X               0
#define A               1
#define B               2
#define Y     	        3
#define LB    	        4
#define RB  	        5
#define LT              6
#define RT              7
#define BACK            8
#define START           9
#define L3              10
#define R3              11

//joystick sticks scale factor
#define scaleFacTrans  50
#define scaleFacRot    10


//----------------------GAIT  MACRO
//TRIPOD
#define tripodAmpWidth 	40 //[mm] amplitude width of leg trajectory for tripod gait 
#define tripodAmpHight	40 //[mm] amplitude hight of leg trajectory for tripod gait	
#define tNumTick        15 //number of trajectory points
//WAVE
#define waveAmpWidth 	40 //[mm] amplitude width of leg trajectory for wave gait
#define waveAmpHight	40 //[mm] amplitude hight of leg trajectory for wave gait
#define wNumTick        15 //number of trajectory points
//RIPPLE
#define rippleAmpWidth 	40 //[mm] amplitude width of leg trajectory for ripple gait
#define rippleAmpHight	40 //[mm] amplitude hight of leg trajectory for ripple gait
#define rNumTick        15 //number of trajectory points


//-------------IF EXPRESSION  MACRO
//jostick activity 
#define MOVING ((pad.speed.x()>0.3)||(pad.speed.x()<-0.3)||(pad.speed.y()>0.3)||(pad.speed.y()<-0.3)||(pad.speed.z()>0.3)||(pad.speed.z()<-0.3))
#define TRANSLATION ((pad.bdT.x()>0.3)||(pad.bdT.x()<-0.3)||(pad.bdT.y()>0.3)||(pad.bdT.y()<-0.3)||(pad.bdT.z()>0.3)||(pad.bdT.z()<-0.3))
#define ROTATION ((pad.bdR.x()>0.3)||(pad.bdR.x()<-0.3)||(pad.bdR.y()>0.3)||(pad.bdR.y()<-0.3)||(pad.bdR.z()>0.3)||(pad.bdR.z()<-0.3))
//joint limits
#define CoxaJointLimit  ((LCS.leg[legNum].jointAngles.alpha<=maxCoxa [legNum])&&(LCS.leg[legNum].jointAngles.alpha>=minCoxa [legNum]))
#define FemurJointLimit ((LCS.leg[legNum].jointAngles.beta <=maxFemur[legNum])&&(LCS.leg[legNum].jointAngles.beta >=minFemur[legNum]))
#define TibiaJointLimit ((LCS.leg[legNum].jointAngles.gamma<=maxTibia[legNum])&&(LCS.leg[legNum].jointAngles.gamma>=minTibia[legNum]))


//----------------GLOBABEL VARIBALE
extern int mode;               //[   MODE   ] -- normal(0)/translation(1)/rotation(2)
extern int gait;               //[   gait   ] -- tripod(1)/wave(2)/ripple(3)
extern int rotBody;            //[ rotBody  ] -- angle of body rotation (0/180)
extern int rollOver;           //[ rollOver ] -- if body roll over (0/1)
extern int ON;                 //[     ON   ] -- if null akrobat shutting down
extern float rollOv[numberOfLegs]; //LCS translational correction after body roll over
extern float rotOfCoxa[numberOfLegs]; //rotates abot coxa for angle 45° init



//--------------------BODY CONSTANT
//body constant initialization
extern float bdConstX[numberOfLegs];//[mm] half hight of body
extern float bdConstY[numberOfLegs];//[mm] half width of body
extern float bdConstZ[numberOfLegs];//[mm] half length of body

//joint angle initialization
extern float jointInitA[numberOfLegs];//[°] (coxa joint) alpha angle init
extern float jointInitB[numberOfLegs];//[°] (femur joint) beta angle init
extern float jointInitC[numberOfLegs];//[°] (tibia joint) gamma angle init

//min limit of coxa joint initialization
extern float minCoxa[numberOfLegs];//[°] (coxa joint) alpha angle min limit
extern float minFemur[numberOfLegs];//[°] (femur joint) beta angle min limit
extern float minTibia[numberOfLegs];//[°] (tibia joint) gamma angle min limit

//max limit of coxa jointinitialization
extern float maxCoxa[numberOfLegs];//[°] (coxa joint) alpha angle max limit
extern float maxFemur[numberOfLegs];//[°] (femur joint) beta angle max limit
extern float maxTibia[numberOfLegs];//[°] (tibia joint) gamma angle max limit


//---------------------------STRUCT
//joypad
struct rumblePad2Struct {
	Vector3 speed; // forward/backward/sideward movement
	Vector3 bdR;   // body rotation
	Vector3 bdT;   // body translation
};
extern rumblePad2Struct rumblePad2;
//trajectory data
struct trajectoryStruct {
	int   caseStep[numberOfLegs];	//leg up/leg down
	int   tick;                     //present tick
	float initAmpX;                 //x init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float initAmpY;                 //y init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float initAmpZ;                 //z init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float ampX[numberOfLegs];       //x axis amplitude of leg trajectory
	float ampY[numberOfLegs];       //y axis amplitude of leg trajectory
	float ampZ[numberOfLegs];       //z axis amplitude of leg trajectory
};
extern trajectoryStruct traData;
// motor data
struct motorStateStruct {
	float timestamp; 	// time stamp
	int   id;           // motor id
	int   goal;         // position value of destination
	int   position;		// present positon
	int   error;		// error of present positon
	int   speed;		// speed to goal positon
	float load;         // currently applied load
	float voltage;		// size of current voltage supplied
	float temperature;	// internal temperature in celsius
	bool  moving;		// goal position execution completed(0)/in progress(1)
};
extern motorStateStruct motorSates[numberOfLegs];
//angles of joint
struct floatJointStruct {
	float alpha;
	float beta;
	float gamma;
};
//leg position
struct legStruct {
	Vector3 footPresPos;            //present position
	Vector3 footInitPos;            //init position
	Vector3 footGlobPos;            //globale position
	Vector3 trajectoryPresPos;      //trajectory present positon
	floatJointStruct jointAngles;	//joint angles
};
//coordinate system
struct coordinateSystemStruct {
	legStruct leg[numberOfLegs];	//array of legs
};

#endif
