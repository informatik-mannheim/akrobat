#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <akrobat/akrobat_init.h>
#include <akrobat/Globals.h>

int gait = 0; // [   gait   ] -- tripod(1)/wave(2)/ripple(3)
int rotBody = 0; // [ rotBody  ] -- angle of body rotation (0/180)
int rollOver = 0; // [ rollOver ] -- if body roll over (0/1)
int ON = 1; // [  ON   ] -- if null akrobat shutting down
float rollOv[numberOfLegs]; // LCS translational correction after body roll over
float rotOfCoxa[numberOfLegs] = { -160, -20, 180, 0, 160, 20 }; // rotates abot coxa for angle 45° init

// body constant initialization
float bdConstX[numberOfLegs] = { -51, 51, -51, 51, -51, 51 }; // [mm] half hight of body
float bdConstY[numberOfLegs] = { 217, 217, 0, 0, -217, -217 }; // [mm] half width of body
float bdConstZ[numberOfLegs] = { 0, 0, 0, 0, 0, 0 }; // [mm] half length of body

// joint angle initialization
float jointInitA[numberOfLegs] = { 160, 20, 180, 0, -160, -20 }; // [°] (coxa joint) alpha angle init
float jointInitB[numberOfLegs] = { 10, 10, 10, 10, 10, 10 }; // [°] (femur joint) beta angle init
float jointInitC[numberOfLegs] = { -90, -90, -90, -90, -90, -90 }; // [°] (tibia joint) gamma angle init
																				  // min limit of coxa joint initialization
float minCoxa[numberOfLegs] = { -26, -71, -51, -51, -71, -23 }; // [°] (coxa joint) alpha angle min limit
float minFemur[numberOfLegs] = { -99, -99, -99, -99, -99, -107 }; // [°] (femur joint) beta angle min limit
float minTibia[numberOfLegs] = { -135, -135, -135, -135, -135, -135 }; // [°] (tibia joint) gamma angle min limit
																				// max limit of coxa jointinitialization
float maxCoxa[numberOfLegs] = { 65, 28, 48, 48, 30, 75 }; // [°] (coxa joint) alpha angle max limit
float maxFemur[numberOfLegs] = { 96, 96, 96, 96, 96, 96 }; // [°] (femur joint) beta angle max limit
float maxTibia[numberOfLegs] = { 135, 135, 135, 135, 135, 135 }; // [°] (tibia joint) gamma angle max limit

rumblePad2Struct pad; // [	PAD   ] -- joypad object
trajectoryStruct traData; // [	.. ] --
coordinateSystemStruct MCS, BCS, LCS, FCS; // [	MCS...] -- coordinate system objects
sensor_msgs::JointState js;