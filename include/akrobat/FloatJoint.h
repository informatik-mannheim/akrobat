#ifndef FLOATJOINT_H
#define FLOATJOINT_H

#include <ostream>

class FloatJoint
{
public:
	float alpha;
	float beta;
	float gamma;

	FloatJoint();
	virtual ~FloatJoint();

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const FloatJoint& b);

#endif // FLOATJOINT_H