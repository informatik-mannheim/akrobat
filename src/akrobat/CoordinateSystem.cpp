/** @file CoordinateSystem.cpp
 *  @brief Responsible for retrieving all leg position in coordinate system.
 *
 *  @author Author
 */

#include <akrobat/CoordinateSystem.h>

/** Reads the information from every leg and writes it to the ostream.
*
*	@param o ostream to write data.
*   @return std::ostream with new leg values.
*/
std::ostream& CoordinateSystem::ToString(std::ostream& o) const
{
	for (int i = 0; i < numberOfLegs; i++)
	{
		o << leg[i] << std::endl;
	}
	return o;
}

/** Overloads the << operator only if it is used with std::ostream and CoordinateSystem as parameters.
*
*	@param o ostream object where the information is written.
*	@param b CoordinateSystem instance.
*   @return std::ostream object with written leg values.
*/
std::ostream& operator<<(std::ostream& o, const CoordinateSystem& b)
{
	return b.ToString(o);
}