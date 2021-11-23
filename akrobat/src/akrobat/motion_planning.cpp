#include <akrobat/Akrobat.h>

using namespace std;


/** Execute the gait functions respective to the current gait mode of the hexapod.
*
*   @return Void.
*/
void Akrobat::runAkrobat()
{
	if (IsMoving() || IsTranslating() || IsRotating())
	{
		jointState.header.stamp = ros::Time::now();


		for (int legNum = 0; legNum < numberOfLegs; legNum++)
		{
			switch (gait)
			{
				case TRIPOD:
				{
					Akrobat::tripodGait(&traData, legNum);
					break;
				}
				case WAVE:
				{
					Akrobat::waveGait(&traData, legNum);
					break;
				}
				case RIPPLE:
				{
					Akrobat::rippleGait(&traData, legNum);
					break;
				}
				default:
					break;
			}

			Akrobat::coordinateTransformation(legNum);
			Akrobat::inverseKinematics(LegCoordinateSystem.leg[legNum].footPresPos.x(), LegCoordinateSystem.leg[legNum].footPresPos.y(), LegCoordinateSystem.leg[legNum].footPresPos.z(), legNum);
			Akrobat::moveLeg(LegCoordinateSystem.leg[legNum].jointAngles.alpha, LegCoordinateSystem.leg[legNum].jointAngles.beta, LegCoordinateSystem.leg[legNum].jointAngles.gamma, legNum);
		}
	}
	else
	{
		Akrobat::moveLeg(-20.0, 10.0, -90, LEFT_FRONT);
		Akrobat::moveLeg(20.0, 10.0, -90, RIGHT_FRONT);
		Akrobat::moveLeg(0.0, 10.0, -90, LEFT_MIDDLE);
		Akrobat::moveLeg(0.0, 10.0, -90, RIGHT_MIDDLE);
		Akrobat::moveLeg(20.0, 10.0, -90, LEFT_REAR);
		Akrobat::moveLeg(-20.0, 10.0, -90, RIGHT_REAR);
	}
	
	jointPub.publish(jointState);
}

/** Create a tripod gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::tripodGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / trajectorySettings[TRIPOD].numTick));
				if ((*tS).tick >= trajectorySettings[TRIPOD].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- down/backward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] - 2.0 * (*tS).ampX[legNum] * (*tS).tick / trajectorySettings[TRIPOD].numTick);
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] - 2.0 * (*tS).ampY[legNum] * (*tS).tick / trajectorySettings[TRIPOD].numTick);
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[TRIPOD].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[TRIPOD].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
}

/** Create a wave gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::waveGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / trajectorySettings[WAVE].numTick));
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- down/backward (1 segment of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * trajectorySettings[WAVE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (*tS).tick / (5.0 * trajectorySettings[WAVE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 3;
				break;

			case 3: // [LEG MOVING] -- down/backward (2 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 4;
				break;

			case 4: // [LEG MOVING] -- down/backward (3 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 5;
				break;

			case 5: // [LEG MOVING] -- down/backward (4 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 6;
				break;

			case 6: // [LEG MOVING] -- down/backward (5 segments of 5)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 4.0 * trajectorySettings[WAVE].numTick) / (5.0 * trajectorySettings[WAVE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[WAVE].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[WAVE].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
}

/** Create a ripple gait for a specific leg
*
*   @param tS includes the data of trajectory for each leg.
*   @param legNum execute the operation for this specific leg.
*
*   @return Void.
*/
void Akrobat::rippleGait(Trajectory* tS, int legNum)
{
	if (IsMoving())
	{
		// [MOVING] --one of joypad sticks was actived
		switch ((*tS).caseStep[legNum])
		{
			case 1: // [LEG MOVING] -- up/forward first half stride
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * (*tS).tick / (2.0 * trajectorySettings[RIPPLE].numTick)));
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 2;
				break;

			case 2: // [LEG MOVING] -- up/forward second half stride
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX(-(*tS).ampX[legNum] * cos(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY(-(*tS).ampY[legNum] * cos(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(abs((*tS).ampZ[legNum]) * sin(M_PI * ((*tS).tick + trajectorySettings[RIPPLE].numTick) / (2.0 * trajectorySettings[RIPPLE].numTick)));
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 3;
				break;

			case 3: // [LEG MOVING] -- down/backward (1 segment of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 4;
				break;

			case 4: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 5;
				break;

			case 5: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 2.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 6;
				break;

			case 6: // [LEG MOVING] -- down/backward (2 segments of 4)
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setX((*tS).ampX[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setY((*tS).ampY[legNum] * (1.0 - 2.0 * (((*tS).tick + 3.0 * trajectorySettings[RIPPLE].numTick) / (4.0 * trajectorySettings[RIPPLE].numTick))));
				FootCoordinateSystem.leg[legNum].trajectoryPresPos.setZ(0);
				if ((*tS).tick >= trajectorySettings[RIPPLE].numTick - 1) (*tS).caseStep[legNum] = 1;
				break;
		}
		if (legNum == numberOfLegs - 1)
		{
			(*tS).tick++;
			if ((*tS).tick > trajectorySettings[RIPPLE].numTick - 1)
			{
				(*tS).tick = 0;
			}
		}
	}
}
