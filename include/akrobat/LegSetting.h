#ifndef LEGSETTING_H
#define LEGSETTING_H

class LegSetting
{
public:
	// body constant initialization
	float bdConstX = 0.0f; // [mm] half hight of body
	float bdConstY = 0.0f; // [mm] half width of body
	float bdConstZ = 0.0f; // [mm] half length of body

	// joint angle initialization
	float jointInitA = 0.0f; // [°] (coxa joint) alpha angle init
	float jointInitB = 0.0f; // [°] (femur joint) beta angle init
	float jointInitC = 0.0f; // [°] (tibia joint) gamma angle init

	// min limit of coxa joint initialization
	float minCoxa = 0.0f; // [°] (coxa joint) alpha angle min limit
	float minFemur = 0.0f; // [°] (femur joint) beta angle min limit
	float minTibia = 0.0f; // [°] (tibia joint) gamma angle min limit

	// max limit of coxa jointinitialization
	float maxCoxa = 0.0f; // [°] (coxa joint) alpha angle max limit
	float maxFemur = 0.0f; // [°] (femur joint) beta angle max limit
	float maxTibia = 0.0f; // [°] (tibia joint) gamma angle max limit
};

#endif // LEGSETTING_H


