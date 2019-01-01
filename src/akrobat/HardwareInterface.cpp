/** @file HardwareInterface.cpp
 *  @brief Responsible for initialization and read/write of the hardware like dynamixel servo motors.
 *
 *  @author Author
 */

#include <akrobat/HardwareInterface.h>
#include <boost/bind.hpp>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

using namespace std;

namespace Akrobat
{
	/** Initialization of the hardware interface. 
	*	Publishes topic for writing hardware and topic for reading hardware.
	*
	*   @return String with the concatenated output values.
	*/
	HardwareInterface::HardwareInterface()
	{
		for (int i = 0, j = 1, k = 1; i < 18; i++)
		{
			std::string name = "m";
			name += ('0' + j);
			name += ('0' + k++);

			if (k > 3)
			{
				j++;
				k = 1;
			}

			command[i] = 0.0;
			publisher[i] = n.advertise<std_msgs::Float64>("/dynamixel_controller/" + name + "/command", 1); // write hardware topic
			subscriber[i] = n.subscribe<dynamixel_msgs::JointState>("/dynamixel_controller/" + name + "/state", 1, boost::bind(&HardwareInterface::readAsync, this, i, _1)); // read hardware topic

			hardware_interface::JointStateHandle state_handle(name, &position[i], &velocity[i], &effort[i]); // read hardware
			jnt_state_interface.registerHandle(state_handle);
			hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(name), &command[i]); // write hardware
			jnt_pos_interface.registerHandle(pos_handle);
		}

		registerInterface(&jnt_state_interface);
		registerInterface(&jnt_pos_interface);
	}

	/** Reads the state of one specific motor and updates the values in the motor array.
	*
	*   @return Void.
	*/
	void HardwareInterface::readAsync(int motorArrayPosition, const dynamixel_msgs::JointStateConstPtr& motorState)
	{
		position[motorArrayPosition] = motorState->current_pos;
		velocity[motorArrayPosition] = motorState->velocity;
		effort[motorArrayPosition] = motorState->load;
	}

	/** Writes new values to ros::Publisher.
	*
	*   @return Void.
	*/
	void HardwareInterface::write()
	{
		std_msgs::Float64 out;

		for (int i = 0; i < numberOfMotors; i++)
		{
			out.data = command[i];
			publisher[i].publish(out);
		}
	}
}