#ifndef LEGSETTING_H
#define LEGSETTING_H

class LegSetting
{
public:
	double rollOv = 0.0;
	double rotOfCoxa = 0.0;
	// body constant initialization
	double bdConstX = 0.0; // [mm] half height of body
	double bdConstY = 0.0; // [mm] half width of body
	double bdConstZ = 0.0; // [mm] half length of body

	// joint angle initialization
	double jointInitA = 0.0; // (coxa joint) alpha angle init
	double jointInitB = 0.0; // (femur joint) beta angle init
	double jointInitC = 0.0; // (tibia joint) gamma angle init

	// min limit of coxa joint initialization
	double minCoxa = 0.0; // (coxa joint) alpha angle min limit
	double minFemur = 0.0; //  (femur joint) beta angle min limit
	double minTibia = 0.0; //  (tibia joint) gamma angle min limit

	// max limit of coxa jointinitialization
	double maxCoxa = 0.0; //  (coxa joint) alpha angle max limit
	double maxFemur = 0.0; // (femur joint) beta angle max limit
	double maxTibia = 0.0; // (tibia joint) gamma angle max limit

	LegSetting();
	LegSetting(double rollOv, double rotOfCoxa, double bdConstX, double bdConstY, double bdConstZ, double jointInitA, double jointInitB, double jointInitC, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia);
};

#endif // LEGSETTING_H