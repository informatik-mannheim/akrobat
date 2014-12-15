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
int mode = 0;               //[   MODE   ] -- normal(0)/translation(1)/rotation(2)
int gait = 0;               //[   gait   ] -- tripod(1)/wave(2)/ripple(3)
int rotBody = 0;            //[ rotBody  ] -- angle of body rotation (0/180)
int rollOver = 0;           //[ rollOver ] -- if body roll over (0/1)
int ON = 1;                 //[     ON   ] -- if null akrobat shutting down
float rollOv[numberOfLegs]; //LCS translational correction after body roll over
float rotOfCoxa[numberOfLegs] =   {  -160,    -20,    180,	0,    160,     20 }; //rotates abot coxa for angle 45° init



//--------------------BODY CONSTANT
//body constant initialization
float bdConstX[numberOfLegs] =    {    -51,     51,     -51,     51,    -51,     51 };//[mm] half hight of body
float bdConstY[numberOfLegs] =    {    217,    217, 	  0,	 0,    -217,   -217 };//[mm] half width of body
float bdConstZ[numberOfLegs] = 	  {      0,      0,       0,     0,       0,      0 };//[mm] half length of body

//joint angle initialization
float jointInitA[numberOfLegs] =  {    160,     20,     180,      0,   -160,    -20  };//[°] (coxa joint) alpha angle init
float jointInitB[numberOfLegs] =  {  	10,  	10,      10,     10,     10,     10  };//[°] (femur joint) beta angle init
float jointInitC[numberOfLegs] =  {    -90,    -90,     -90,    -90,    -90,    -90  };//[°] (tibia joint) gamma angle init

//min limit of coxa joint initialization
float minCoxa[numberOfLegs] =	  {    -26,    -71,    -51,    -51,    -71,    -23  };//[°] (coxa joint) alpha angle min limit
float minFemur[numberOfLegs] =	  {    -99,    -99,    -99,    -99,    -99,   -107  };//[°] (femur joint) beta angle min limit
float minTibia[numberOfLegs] =	  {   -135,   -135,   -135,   -135,   -135,   -135  };//[°] (tibia joint) gamma angle min limit

//max limit of coxa jointinitialization
float maxCoxa[numberOfLegs] = 	  {     65,     28,     48,     48,     30,     75  };//[°] (coxa joint) alpha angle max limit
float maxFemur[numberOfLegs] =	  {     96,     96,     96,     96,     96,     96  };//[°] (femur joint) beta angle max limit
float maxTibia[numberOfLegs] =	  {    135,    135,    135,    135,    135,    135  };//[°] (tibia joint) gamma angle max limit


//---------------------------STRUCT
//joypad
struct rumblePad2Struct{
	Vector3 speed; // forward/backward/sideward movement
	Vector3 bdR;   // body rotation
	Vector3 bdT;   // body translation
};
extern rumblePad2Struct rumblePad2;
//trajectory data
struct trajectoryStruct{
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
struct motorStateStruct{
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
struct floatJointStruct{
	float alpha;
	float beta;
	float gamma;
};
//leg position
struct legStruct{
    Vector3 footPresPos;            //present position
    Vector3 footInitPos;            //init position
    Vector3 footGlobPos;            //globale position
    Vector3 trajectoryPresPos;      //trajectory present positon
	floatJointStruct jointAngles;	//joint angles
};
//coordinate system
struct coordinateSystemStruct{
	legStruct leg[numberOfLegs];	//array of legs
};


//----------------------------CLASS
class Akrobat{

  private:
    NodeHandle n;
    Subscriber subJoy;          //subscriber of joy topic
    Publisher jointPub;         //publisher (rviz)
   
    //LEG1
    Publisher  pubLeg1Joint1;	//publicher for jointX of legX
    Publisher  pubLeg1Joint2;	// .....
    Publisher  pubLeg1Joint3;
    //LEG2
    Publisher  pubLeg2Joint1;
    Publisher  pubLeg2Joint2;
    Publisher  pubLeg2Joint3;
    //LEG3
    Publisher  pubLeg3Joint1;
    Publisher  pubLeg3Joint2;
    Publisher  pubLeg3Joint3;
    //LEG4
    Publisher  pubLeg4Joint1;
    Publisher  pubLeg4Joint2;
    Publisher  pubLeg4Joint3;
    //LEG5
    Publisher  pubLeg5Joint1;
    Publisher  pubLeg5Joint2;
    Publisher  pubLeg5Joint3;
    //LEG6
    Publisher  pubLeg6Joint1;
    Publisher  pubLeg6Joint2;
    Publisher  pubLeg6Joint3;   // .....

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
    Transform transformCS(string sourceCS, string targetCS, Vector3 rot, Vector3 trans);
 
    //call the motor state list back
    void callRumblePad2Back(const sensor_msgs::Joy::ConstPtr& joy);
};


#endif
