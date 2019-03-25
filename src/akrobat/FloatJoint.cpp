/** @file FloatJoint.cpp
 *  @brief Joins and writes leg angles alpha, beta, gamma into ostream.
 *
 *  @author Author
 */

#include <akrobat/FloatJoint.h>

/** Receives an ostream object and writes the joined leg angles alpha, beta and gamma from one specific leg into it.
*
*	@param o ostream object.
*   @return std::ostream object with new values.
*/
std::ostream& FloatJoint::ToString(std::ostream& o) const
{
	return o << alpha << "," << beta << "," << gamma;
}