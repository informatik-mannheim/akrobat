#ifndef LEGSTRUCT_H
#define LEGSTRUCT_H

#include <tf/transform_datatypes.h>

#include <akrobat/FloatJointStruct.h>

struct legStruct
{
	tf::Vector3 footPresPos; // present position
	tf::Vector3 footInitPos; // init position
	tf::Vector3 footGlobPos; // globale position
	tf::Vector3 trajectoryPresPos; // trajectory present positon
	floatJointStruct jointAngles; // joint angles

	legStruct() :footPresPos(0.0, 0.0, 0.0), footInitPos(0.0, 0.0, 0.0), footGlobPos(0.0, 0.0, 0.0), trajectoryPresPos(0.0, 0.0, 0.0)
	{
	}
};

#endif // LEGSTRUCT_H