#ifndef TRAJECTORYSETTINGS_H
#define TRAJECTORYSETTINGS_H

class TrajectorySettings
{
public:
	int ampWidth;
	int ampHight;
	int numTick;

	TrajectorySettings();
	TrajectorySettings(int ampWidth, int ampHight, int numTick);
	virtual ~TrajectorySettings();
};

#endif // TRAJECTORYSETTINGS_H