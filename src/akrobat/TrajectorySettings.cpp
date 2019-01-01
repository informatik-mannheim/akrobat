/** @file TrajectorySettings.cpp
 *  @brief Class for managing trajectory settings.
 *
 *  @author Author
 */

#include <akrobat/TrajectorySettings.h>

/** Default constructor for the TrajectorySettings class.
* 
*/
TrajectorySettings::TrajectorySettings() :ampWidth(0), ampHight(0), numTick(0){}

/** Overload constructor for the TrajectorySettings class with custom initialize values.
*   @param ampWidth ???.
*   @param ampHight ???.
*   @param numTick ???.
*/
TrajectorySettings::TrajectorySettings(int ampWidth, int ampHight, int numTick):ampWidth(ampWidth), ampHight(ampHight), numTick(numTick){}

TrajectorySettings::~TrajectorySettings() {}