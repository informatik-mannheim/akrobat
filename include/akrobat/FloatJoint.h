#ifndef FLOATJOINT_H
#define FLOATJOINT_H

#include <ostream>

class FloatJoint
{
public:
	float alpha = 0.0f;
	float beta = 0.0f;
	float gamma = 0.0f;

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const FloatJoint& b);

#endif // FLOATJOINT_H