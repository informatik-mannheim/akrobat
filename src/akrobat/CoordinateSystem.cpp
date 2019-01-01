/** @file CoordinateSystem.cpp
 *  @brief Initialization file for Controller.
 *
 *  @author Author
 */

#include <akrobat/CoordinateSystem.h>

/** Keine Ahnung.
*
*   @return ???.
*/
std::ostream& CoordinateSystem::ToString(std::ostream& o) const
{
	for (int i = 0; i < numberOfLegs; i++)
	{
		o << leg[i] << std::endl;
	}

	return o;
}

/** Unnötig? Kann doch direkt in der Methode obendrüber gemacht werden.
*
*   @return ???.
*/
std::ostream& operator<<(std::ostream& o, const CoordinateSystem& b)
{
	return b.ToString(o);
}