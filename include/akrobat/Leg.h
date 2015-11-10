#ifndef LEG_H
#define LEG_H

#include <tf/transform_datatypes.h>

#include <akrobat/FloatJoint.h>

class Leg
{
public:
	tf::Vector3 footPresPos; // present position
	tf::Vector3 footInitPos; // init position
	tf::Vector3 footGlobPos; // globale position
	tf::Vector3 trajectoryPresPos; // trajectory present positon
	FloatJoint jointAngles; // joint angles

	Leg();
};

#endif // LEG_H