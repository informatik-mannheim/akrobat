#include <akrobat/FloatJoint.h>

std::ostream& FloatJoint::ToString(std::ostream& o) const
{
	return o << alpha << "," << beta << "," << gamma;
}