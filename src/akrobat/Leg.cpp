/** @file Leg.cpp
 *  @brief Handles Leg status and positions.
 *
 *  @author Author
 */

#include <akrobat/Leg.h>
#include <iomanip>

/** Writes the different foot positions (present, init, global, trajectory present) of a leg into the ostream.
*
*	@param o ostream object for writing foot positions.
*   @return std::ostream.
*/
std::ostream& Leg::ToString(std::ostream& o) const
{
	return o
		<< "footPresPos: " << footPresPos << std::endl
		<< "footInitPos: " << footInitPos << std::endl
		<< "footGlobPos: " << footGlobPos << std::endl
		<< "trajectoryPresPos: " << trajectoryPresPos << std::endl;
}

/** Overloads the << operator only if it is used with std::ostream and Leg as parameters.
*
*	@param o ostream object.
*	@param b Leg instance on which the operation is running.
*   @return std::ostream.
*/
std::ostream& operator<<(std::ostream& o, const Leg& b)
{
	return b.ToString(o);
}

/** Overloads the << operator only if it is used with std::ostream and tf::Vector3 as parameters.
* 
*	@param o ostream object.
*	@param vector3 Vector for different type of leg position (present, init, global, trajectory present).
*   @return std::ostream.
*/
std::ostream& operator<<(std::ostream& o, const tf::Vector3& vector3)
{
	// Save flags/precision.
	std::ios_base::fmtflags oldflags = o.flags();
	std::streamsize oldprecision = o.precision();

	o << std::fixed << std::setprecision(2)
		<< vector3.x() << ", " << vector3.y() << ", " << vector3.z();

	// Restore flags/precision.
	o.flags(oldflags);
	o.precision(oldprecision);

	return o;
}