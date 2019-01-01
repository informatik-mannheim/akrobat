/** @file LegSettings.cpp
 *  @brief Sets initial leg settings..
 *
 *  @author Author
 */

#include <akrobat/LegSetting.h>

LegSetting::LegSetting() {}

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