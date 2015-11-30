#include <akrobat/CoordinateSystem.h>

std::ostream& CoordinateSystem::ToString(std::ostream& o) const
{
	for (int i = 0; i < numberOfLegs; i++)
	{
		o << leg[i] << std::endl;
	}

	return o;
}

std::ostream& operator<<(std::ostream& o, const CoordinateSystem& b)
{
	return b.ToString(o);
}