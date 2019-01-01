/** @file FloatJoint.cpp
 *  @brief Concatenades float numbers alpha, beta and gamma into one string.
 *
 *  @author Author
 */

#include <akrobat/FloatJoint.h>

/** Receives the information from output stream and concatenates alpha, beta and gamma values.
*
*   @return String with the concatenated output values.
*/
std::ostream& FloatJoint::ToString(std::ostream& o) const
{
	return o << alpha << "," << beta << "," << gamma;
}