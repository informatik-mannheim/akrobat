#ifndef LEGSETTING_H
#define LEGSETTING_H

class LegSetting
{
public:
	double rollOv;
	double rotOfCoxa;
	// body constant initialization
	double bdConstX; // [mm] half hight of body
	double bdConstY; // [mm] half width of body
	double bdConstZ; // [mm] half length of body

	// joint angle initialization
	double jointInitA; // [°] (coxa joint) alpha angle init
	double jointInitB; // [°] (femur joint) beta angle init
	double jointInitC; // [°] (tibia joint) gamma angle init

	// min limit of coxa joint initialization
	double minCoxa; // [°] (coxa joint) alpha angle min limit
	double minFemur; // [°] (femur joint) beta angle min limit
	double minTibia; // [°] (tibia joint) gamma angle min limit

	// max limit of coxa jointinitialization
	double maxCoxa; // [°] (coxa joint) alpha angle max limit
	double maxFemur; // [°] (femur joint) beta angle max limit
	double maxTibia; // [°] (tibia joint) gamma angle max limit

	LegSetting();
	LegSetting(double rollOv, double rotOfCoxa, double bdConstX, double bdConstY, double bdConstZ, double jointInitA, double jointInitB, double jointInitC, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia);
};

#endif // LEGSETTING_H