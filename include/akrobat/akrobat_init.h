#ifndef AKROBAT_INIT_H
#define AKROBAT_INIT_H

/*
TODO:
	- Laufmuster wechsel/ eine taste pro muster
	-
	[walk_x, walk_y, curve_left, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]
	[stop, walk_tripod, walk_wave, walk_ripple, walk_insect, walk_planner, turn_upside_down] // control mode angedacht
																							 // shitf1, shift2 ändert das remapping
																							 // auf verschiedne topics aufbrechen [laufen][rotieren][cam]

	[walk_x, walk_y, curve_left, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]
	[stop, walk_tripod, walk_wave, walk_ripple, walk_insect, walk_planner, turn_upside_down]

*/

//---------------------OUTPUT MACRO
#define  F1DEBUG 	0 //initAkrobat()
#define  F2DEBUG 	0 //runAkrobat()
#define  F3DEBUG 	0 //tripodGait()
#define  F4DEBUG 	0 //waveGait()
#define  F5DEBUG 	0 //rippleGait()
#define  F6DEBUG 	0 //coordinateTransformation()
#define  F7DEBUG 	0//inverseKinematics()
#define  F8DEBUG 	0 //moveLeg()
#define  F9DEBUG 	0 //transformCS()
#define  F10DEBUG 	0 //callRumblePad2Back()

//-----------------------BODY MACRO
//#define halfBodyHight  	0     //[mm] half hight of body
//#define halfBodyWidth  	51    //[mm] half width of body
//#define halfBodyLength 	217   //[mm] half length of body

//------------------------LEG MACRO
#define LEFT_FRONT    	0 //[ID'S] COXA: 11 / FEMUR: 12 / TIBIA: 13
#define RIGHT_FRONT     1 //[ID'S] COXA: 21 / FEMUR: 22 / TIBIA: 23
#define LEFT_MIDDLE     2 //[ID'S] COXA: 31 / FEMUR: 32 / TIBIA: 33
#define RIGHT_MIDDLE	3 //[ID'S] COXA: 41 / FEMUR: 42 / TIBIA: 43
#define LEFT_REAR       4 //[ID'S] COXA: 51 / FEMUR: 52 / TIBIA: 53
#define RIGHT_REAR      5 //[ID'S] COXA: 61 / FEMUR: 62 / TIBIA: 63

//#define numberOfJoints  3
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

// TODO check joy device
// mode cannot be detected // mode turns left stick into digital input mode
// D-X switch can be identified with number of axis available

#define X_BUTTON               0
#define A_BUTTON               1
#define B_BUTTON               2
#define Y_BUTTON     	        3
#define LB_BUTTON    	        4
#define RB_BUTTON  	        5
#define LT_BUTTON              6
#define RT_BUTTON              7
#define BACK_BUTTON            8
#define START_BUTTON           9
#define L3_BUTTON              10
#define R3_BUTTON              11

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

#endif

