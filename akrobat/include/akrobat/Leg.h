#ifndef LEG_H
#define LEG_H

#include <ostream>

#include <tf/transform_datatypes.h>

#include <akrobat/FloatJoint.h>

class Leg
{
public:
	tf::Vector3 footPresPos = tf::Vector3(0.0, 0.0, 0.0); // present position
	tf::Vector3 footInitPos = tf::Vector3(0.0, 0.0, 0.0); // init position
	tf::Vector3 footGlobPos = tf::Vector3(0.0, 0.0, 0.0); // globale position
	tf::Vector3 trajectoryPresPos = tf::Vector3(0.0, 0.0, 0.0); // trajectory present positon
	FloatJoint jointAngles; // joint angles

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const Leg& b);

std::ostream& operator<<(std::ostream& stream, const tf::Vector3& vector3);
#endif