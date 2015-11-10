#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <akrobat/akrobat_init.h>

class Trajectory
{
public:
	int caseStep[numberOfLegs]; //leg up/leg down
	int tick; //present tick
	float initAmpX; //x init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float initAmpY; //y init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float initAmpZ; //z init amplitude (tripodAmpWidth/waveAmpWidth/rippleAmpWidth)
	float ampX[numberOfLegs]; //x axis amplitude of leg trajectory
	float ampY[numberOfLegs]; //y axis amplitude of leg trajectory
	float ampZ[numberOfLegs]; //z axis amplitude of leg trajectory

	Trajectory();
};

#endif // TRAJECTORY_H