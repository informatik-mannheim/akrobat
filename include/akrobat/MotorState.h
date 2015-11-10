#ifndef MOTORSTATE_H
#define MOTORSTATE_H

class MotorState
{
public:
	float timestamp; // time stamp
	int id; // motor id
	int goal; // position value of destination
	int position; // present positon
	int error; // error of present positon
	int speed; // speed to goal positon
	float load; // currently applied load
	float voltage; // size of current voltage supplied
	float temperature; // internal temperature in celsius
	bool moving; // goal position execution completed(0)/in progress(1)

	MotorState();
};

#endif // MOTORSTATE_H

