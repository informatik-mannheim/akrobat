#ifndef LEG_H
#define LEG_H

#include <ostream>

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

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const Leg& b);

std::ostream& operator<<(std::ostream& stream, const tf::Vector3& vector3);
#endif // LEG_H