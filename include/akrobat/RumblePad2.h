#ifndef RUMBLEPAD2_H
#define RUMBLEPAD2_H

#include <tf/transform_datatypes.h>

class RumblePad2
{
public:
	tf::Vector3 speed; // forward/backward/sideward movement
	tf::Vector3 bdR; // body rotation
	tf::Vector3 bdT; // body translation

	RumblePad2();
};

#endif // RUMBLEPAD2_H


