/** @file TrajectorySettings.cpp
 *  @brief Class for managing trajectory settings.
 *
 *  @author Author
 */

#include <akrobat/TrajectorySettings.h>

/** Overloads TrajectorySettings constructor.
* 
*/
TrajectorySettings::TrajectorySettings():
ampWidth(0),
ampHight(0),
numTick(0)
{}

/** Overload constructor for the TrajectorySettings class with custom initialize values.
*   @param ampWidth [mm] X amplitude width of leg trajectory for tripod gait.
*   @param ampHight [mm] Z amplitude hight of leg trajectory for tripod gait.
*   @param numTick Ticker of one cycle for the servo motors.
*/
TrajectorySettings::TrajectorySettings(int ampWidth, int ampHight, int numTick):
ampWidth(ampWidth),
ampHight(ampHight),
numTick(numTick)
{}

/** Destructor for TrajectorySettings class.
* 
*/
TrajectorySettings::~TrajectorySettings() {}