#ifndef LEGSETTING_H
#define LEGSETTING_H

class LegSetting
{
public:
	// body constant initialization
	float bdConstX; // [mm] half hight of body
	float bdConstY; // [mm] half width of body
	float bdConstZ; // [mm] half length of body

	// joint angle initialization
	float jointInitA; // [°] (coxa joint) alpha angle init
	float jointInitB; // [°] (femur joint) beta angle init
	float jointInitC; // [°] (tibia joint) gamma angle init

	// min limit of coxa joint initialization
	float minCoxa; // [°] (coxa joint) alpha angle min limit
	float minFemur; // [°] (femur joint) beta angle min limit
	float minTibia; // [°] (tibia joint) gamma angle min limit

	// max limit of coxa jointinitialization
	float maxCoxa; // [°] (coxa joint) alpha angle max limit
	float maxFemur; // [°] (femur joint) beta angle max limit
	float maxTibia; // [°] (tibia joint) gamma angle max limit

	LegSetting(float bdConstX = 0.0f, float bdConstY = 0.0f, float bdConstZ = 0.0f, float jointInitA = 0.0f, float jointInitB = 0.0f, float jointInitC = 0.0f, float minCoxa = 0.0f, float minFemur = 0.0f, float minTibia = 0.0f, float maxCoxa = 0.0f, float maxFemur = 0.0f, float maxTibia = 0.0f) :
		bdConstX(bdConstX),
		bdConstY(bdConstY),
		bdConstZ(bdConstZ),
		jointInitA(jointInitA),
		jointInitB(jointInitB),
		jointInitC(jointInitC),
		minCoxa(minCoxa),
		minFemur(minFemur),
		minTibia(minTibia),
		maxCoxa(maxCoxa),
		maxFemur(maxFemur),
		maxTibia(maxTibia)
	{
	}
};

#endif // LEGSETTING_H


