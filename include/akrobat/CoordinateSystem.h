#ifndef COORDINATESYSTEM_H
#define COORDINATESYSTEM_H

#include <akrobat/Settings.h>
#include <akrobat/Leg.h>

class CoordinateSystem
{
public:
	Leg leg[numberOfLegs];

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const CoordinateSystem& b);

#endif