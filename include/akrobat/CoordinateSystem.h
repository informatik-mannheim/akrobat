#ifndef COORDINATESYSTEM_H
#define COORDINATESYSTEM_H

#include <akrobat/akrobat_init.h>
#include <akrobat/Leg.h>

class CoordinateSystem
{
public:
	Leg leg[numberOfLegs]; //array of legs

	virtual std::ostream& ToString(std::ostream& o) const;
};

std::ostream& operator<<(std::ostream& o, const CoordinateSystem& b);

#endif // COORDINATESYSTEM_H