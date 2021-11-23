/** @file LegSettings.cpp
 *  @brief Configuration for initial leg settings.
 *
 *  @author Author
 */

#include <akrobat/LegSetting.h>

/** LegSetting constructor.
*/
LegSetting::LegSetting() {}

/** Overlaods LegSetting constructor.
*	@param rollOv Initial value for roll over of the leg.
*	@param rotOfCoxa initial rotation of coxa.
*	@param bdConstX [mm] half height of body
*	@param bdConstY [mm] half width of body
*	@param bdConstZ [mm] half length of body
*	@param jointInitA (coxa joint) alpha angle init
*	@param jointInitB (femur joint) beta angle init
*	@param jointInitC (tibia joint) gamma angle init
*	@param minCoxa (coxa joint) alpha angle min limit
*	@param minFemur (femur joint) beta angle min limit
*	@param minTibia (tibia joint) gamma angle min limit
*	@param maxCoxa (coxa joint) alpha angle max limit
*	@param maxFemur (femur joint) beta angle max limit
*	@param maxTibia (tibia joint) gamma angle max limit
*/
LegSetting::LegSetting(double rollOv, double rotOfCoxa, double bdConstX, double bdConstY, double bdConstZ, double jointInitA, double jointInitB, double jointInitC, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia) :
	rollOv(rollOv),
	rotOfCoxa(rotOfCoxa),
	bdConstX(bdConstX),
	bdConstY(bdConstY),
	bdConstZ(bdConstZ),
	jointInitA(jointInitA),
	jointInitB(jointInitB),
	jointInitC(jointInitC),
	minCoxa(minCoxa),
	minFemur(minFemur),
	minTibia(minTibia),
	maxCoxa(maxCoxa),
	maxFemur(maxFemur),
	maxTibia(maxTibia)
{}