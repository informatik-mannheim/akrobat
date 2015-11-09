#ifndef LEGSETTING_H
#define LEGSETTING_H

class LegSetting
{
public:
	float rollOv;
	float rotOfCoxa;
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

	LegSetting()
	{
		rollOv = 0.0f;
		rotOfCoxa = 0.0f;

		bdConstX = 0.0f;
		bdConstY = 0.0f;
		bdConstZ = 0.0f;

		jointInitA = 0.0f;
		jointInitB = 0.0f;
		jointInitC = 0.0f;

		minCoxa = 0.0f;
		minFemur = 0.0f;
		minTibia = 0.0f;

		maxCoxa = 0.0f;
		maxFemur = 0.0f;
		maxTibia = 0.0f;
	}

	LegSetting(float rollOv, float rotOfCoxa, float bdConstX, float bdConstY, float bdConstZ, float jointInitA, float jointInitB, float jointInitC, float minCoxa, float minFemur, float minTibia, float maxCoxa, float maxFemur, float maxTibia) :
		rollOv(rollOv),
		rotOfCoxa(rotOfCoxa),
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