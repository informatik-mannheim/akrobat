#include <akrobat/FloatJoint.h>

FloatJoint::FloatJoint() : alpha(0.0f), beta(0.0f), gamma(0.0f)
{
}

FloatJoint::~FloatJoint()
{
}

std::ostream& FloatJoint::ToString(std::ostream& o) const
{
	return o << alpha << "," << beta << "," << gamma;
}