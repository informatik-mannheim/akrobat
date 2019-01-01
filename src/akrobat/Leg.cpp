/** @file Leg.cpp
 *  @brief Responsible for leg status.
 *
 *  @author Author
 */

#include <akrobat/Leg.h>
#include <iomanip>

/** ???.
*
*   @return String.
*/
std::ostream& Leg::ToString(std::ostream& o) const
{
	return o
		<< "footPresPos: " << footPresPos << std::endl
		<< "footInitPos: " << footInitPos << std::endl
		<< "footGlobPos: " << footGlobPos << std::endl
		<< "trajectoryPresPos: " << trajectoryPresPos << std::endl;
}

/** Converts information about one leg to string.
*
*   @return String.
*/
std::ostream& operator<<(std::ostream& o, const Leg& b)
{
	return b.ToString(o);
}

/** ???.
*
*   @return stream.
*/
std::ostream& operator<<(std::ostream& stream, const tf::Vector3& vector3)
{
	// Save flags/precision.
	std::ios_base::fmtflags oldflags = stream.flags();
	std::streamsize oldprecision = stream.precision();

	stream << std::fixed << std::setprecision(2)
		<< vector3.x() << ", " << vector3.y() << ", " << vector3.z();

	// Restore flags/precision.
	stream.flags(oldflags);
	stream.precision(oldprecision);

	return stream;
}