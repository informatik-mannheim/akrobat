#include <akrobat/MotorState.h>

MotorState::MotorState() :
	timestamp(0.0f),
	id(0),
	goal(0),
	position(0),
	error(0),
	speed(0),
	load(0.0f),
	voltage(0.0f),
	temperature(0.0f),
	moving(false)
{
}