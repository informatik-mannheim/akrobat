#ifndef GLOBALS_H
#define GLOBALS_H

#include <akrobat/akrobat_init.h>
#include <akrobat/RumblePad2Struct.h>
#include <akrobat/TrajectoryStruct.h>
#include <akrobat/MotorStateStruct.h>
#include <akrobat/CoordinateSystemStruct.h>

// ----------------GLOBABEL VARIBALE
extern int mode; // [   MODE   ] -- normal(0)/translation(1)/rotation(2)
extern int gait; // [   gait   ] -- tripod(1)/wave(2)/ripple(3)
extern int rotBody; // [ rotBody  ] -- angle of body rotation (0/180)
extern int rollOver; // [ rollOver ] -- if body roll over (0/1)
extern int ON; // [     ON   ] -- if null akrobat shutting down
extern float rollOv[numberOfLegs]; // LCS translational correction after body roll over
extern float rotOfCoxa[numberOfLegs]; // rotates abot coxa for angle 45° init

// --------------------BODY CONSTANT
// body constant initialization
extern float bdConstX[numberOfLegs]; // [mm] half hight of body
extern float bdConstY[numberOfLegs]; // [mm] half width of body
extern float bdConstZ[numberOfLegs]; // [mm] half length of body

// joint angle initialization
extern float jointInitA[numberOfLegs]; // [°] (coxa joint) alpha angle init
extern float jointInitB[numberOfLegs]; // [°] (femur joint) beta angle init
extern float jointInitC[numberOfLegs]; // [°] (tibia joint) gamma angle init

// min limit of coxa joint initialization
extern float minCoxa[numberOfLegs]; // [°] (coxa joint) alpha angle min limit
extern float minFemur[numberOfLegs]; // [°] (femur joint) beta angle min limit
extern float minTibia[numberOfLegs]; // [°] (tibia joint) gamma angle min limit

// max limit of coxa jointinitialization
extern float maxCoxa[numberOfLegs]; // [°] (coxa joint) alpha angle max limit
extern float maxFemur[numberOfLegs]; // [°] (femur joint) beta angle max limit
extern float maxTibia[numberOfLegs]; // [°] (tibia joint) gamma angle max limit

extern rumblePad2Struct rumblePad2;
extern trajectoryStruct traData;
extern motorStateStruct motorSates[numberOfLegs];

extern rumblePad2Struct pad;                           //[	PAD   ] -- joypad object
extern trajectoryStruct traData;                       //[	..    ] --
extern coordinateSystemStruct MCS, BCS, LCS, FCS;      //[	MCS...] -- coordinate system objects
extern sensor_msgs::JointState js;
#endif //  GLOBALS_H