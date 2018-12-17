#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "akrobat/movement.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

float int_max = 32767.0f;
float joystickDeadZone = 0.2f;
enum Mode_right_joystick 
{
	DEFAULT_MODE, 
	shift, 
	roll_pitch, 
	yaw, 
	level
};
enum Walking_Mode 
{
	tripod, 
	wave, 
	ripple
};
enum Mode 
{
	navigate, 
	work
};
enum Macro 
{
	DEFAULT_MACRO, 
	feet,
	orientate_shift,
	orientate_yaw,
	orientate_level,
	orientate_roll_pitch,
	start,
	shutdown
};

class Listener
{
	public:
	std::vector<float> getAxisValue()
	{
		return axis;
	}

	void setAxisValue(std::vector<float> av)
	{
		axis = av;
	}

	std::vector<int> getButtonsValue()
	{
		return buttons;
	}

	void setButtonsValue(std::vector<int> bv)
	{
		buttons = bv;
	}

	std::string getControllerType()
	{
		return controller_type;
	}

	void setControllerType(std::string ct)
	{
		controller_type = ct;
	}

	void readJoypadCallback(const sensor_msgs::Joy::ConstPtr& msg)
	{
		amountAxis = msg->axes.size();
   	amountButtons = msg->buttons.size();
		std::cout << "Anzahl Achsen: " << amountAxis << "\n Anzahl Buttons:" << amountButtons << std::endl;
		if(amountAxis == 8 && amountButtons == 11)
		{
			/*
				Achsennummern:								Buttonnummern:
				0: Joystick links, rechts<->links	0: A
				1: Joystick links, oben<->unten		1: B
				2: Joystick rechts, rechts<->links	2: X
				3: Joystick rechts, oben<->unten		3: Y
				4: LT											4: LB
				5: RT											5: RB
				6:	Dig. Joystick, rechts<->links		6: BACK
				7: Dig. Joystick, oben<->unten		7: START
																8: LOGITECH
																9: LJoystick (drücken)
																10: RJoystick (drücken)
			*/
			setControllerType("Logitech Gamepad F710");
		}
		setAxisValue(msg->axes);
		setButtonsValue(msg->buttons);
	}

	protected:
	std::vector<float> axis;
	std::vector<int> buttons;
	std::string controller_type = "DEFAULT!";
	int amountAxis = -1;
	int amountButtons = -1;
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "translateJoypad");
   ros::NodeHandle n;
	Listener l;
   ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &Listener::readJoypadCallback, &l);
   ros::Publisher movPub = n.advertise<akrobat::movement>("movements", 1000);
   ros::Rate loop_rate(5);
	Mode mode = navigate;
	Walking_Mode wmode = tripod;
	Mode_right_joystick m = DEFAULT_MODE;
	Macro macro = start;
	std::vector<int> buttonsPressed = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

   while(ros::ok())
   {
		std::vector<float> axisValue = l.getAxisValue();
		std::vector<int> buttonsValue = l.getButtonsValue();
		std::string ct = l.getControllerType();
      akrobat::movement msg;
		
		//Überprüfen, ob der richtige Controller angeschlossen ist
		if(ct != "Logitech Gamepad F710")
		{
			std::cout << "Falscher Controller!!!" << std::endl;
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		float commands[9] = {0.0f}; //w_x, w_y, w_alpha, yaw, pitch, roll, m_x, m_y, m_z

		//Befehle im Navigationsmodus
		if(mode == navigate)
		{
			//LINKER JOYSTICK
			if(std::abs(axisValue[1]) > joystickDeadZone)
			{ 
				commands[0] = axisValue[1]*int_max;
			}
			else
			{
				commands[0] = 0;
			}
			if(std::abs(axisValue[0]) > joystickDeadZone)
			{
				commands[1] = axisValue[0]*int_max*(-1);
			}
			else
			{
				commands[1] = 0;
			}

			//RECHTER JOYSTICK
			if(m != DEFAULT_MODE)
			{
				if(m == shift)
				{
					if(std::abs(axisValue[4]) > joystickDeadZone)
					{
						commands[6] = axisValue[4]*int_max;
					}
					else
					{
						commands[6] = 0;
					}
					if(std::abs(axisValue[3]) > joystickDeadZone)
					{					
						commands[7] = axisValue[3]*int_max;
					}
					else
					{
						commands[7] = 0;
					}
				}
				else if(m == roll_pitch)
				{
					if(std::abs(axisValue[4]) > joystickDeadZone)
					{
						commands[4] = axisValue[4]*int_max;
					}
					else
					{
						commands[4] = 0;
					}
					if(std::abs(axisValue[3]) > joystickDeadZone)
					{
						commands[5] = axisValue[3]*int_max;
					}
					else
					{
						commands[5] = 0;
					}
				}
				else if(m == yaw)
				{
					if(std::abs(axisValue[3]) > joystickDeadZone)
					{
						commands[3] = axisValue[3]*int_max;
					}
					else
					{
						commands[3] = 0;
					}
				}
				else if(m == level)
				{
					if(std::abs(axisValue[4]) > joystickDeadZone)
					{
						commands[8] = axisValue[4]*int_max;
					}
					else
					{
						commands[8] = 0;
					}
				}
			}
	
			//RT&LT
			if(std::abs(axisValue[2] - axisValue[5]) > joystickDeadZone)
			{
				commands[2] = ((axisValue[2]-axisValue[5])*int_max)/2;
				//Im Uhrzeigersinn drehen --> positiv
			}
			else
			{
				commands[2] = 0;
			}

			//Dig. Joystick
			if(axisValue[7] > 0)
			{
				macro = feet;
			}
			else
			{
				macro = DEFAULT_MACRO;
			}
			
			//Button A
			if(buttonsValue[0] == 1)
			{
				m = shift;
			}

			//Button B
			if(buttonsValue[1] == 1)
			{
				m = roll_pitch;
			}

			//Button X
			if(buttonsValue[2] == 1)
			{
				m = level;
			}

			//Button Y
			if(buttonsValue[3] == 1)
			{
				m = yaw;
			}

			//Button LB
			if(buttonsValue[4] == 1)
			{
				wmode = static_cast<Walking_Mode>((wmode + 1) % 3);
			}

			//Button RB
			//NOTHING TO DO HERE

			//Button START
			if(buttonsValue[7] == 1)
			{
				macro = start;
			}

			//Button BACK
			if(buttonsValue[6] == 1)
			{
				mode = static_cast<Mode>((mode + 1) % 2);
			}

			//Button Logitech
			//NOTHING TO DO HERE

			//Button LJoystick && Button RJoystick
			if(buttonsValue[9] == 1 && buttonsValue[10] == 1)
			{
				macro = shutdown;
			}
			
		}

		//Befehle im Arbeitsmodus
		else if(mode == work)
		{
			//LINKER JOYSTICK
			if(std::abs(axisValue[1]) > joystickDeadZone)
			{
				commands[6] = axisValue[1]*int_max;
			}		
			else
			{
				commands[6] = 0;
			}		
			if(std::abs(axisValue[0]) > joystickDeadZone)
			{
				commands[7] = axisValue[0]*int_max*(-1);
			}
			else
			{
				commands[7] = 0;
			}

			//RECHTER JOYSTICK
			if(std::abs(axisValue[4]) > joystickDeadZone)
			{			
				commands[4] = axisValue[4]*int_max;
			}
			else
			{
				commands[4] = 0;
			}
			if(std::abs(axisValue[3]) > joystickDeadZone)
			{			
				commands[5] = axisValue[3]*int_max*(-1);
			}
			else
			{
				commands[5] = 0;
			}

			//RT&LT
			if(std::abs(axisValue[2] - axisValue[5]) > joystickDeadZone)
			{
				commands[3] = ((axisValue[2]-axisValue[5])*int_max)/2;
				//Im Uhrzeigersinn --> positiv
			}
			else
			{
				commands[3] = 0;
			}

			//DIGITALER JOYSTICK
			//NOTHING TO DO HERE

			//BUTTON A
			//NOTHING TO DO HERE

			//BUTTON B
			//NOTHING TO DO HERE

			//BUTTON X
			//NOTHING TO DO HERE

			//BUTTON Y
			//NOTHING TO DO HERE

			//BUTTON LB&RB
			commands[8] = ((buttonsValue[5]-buttonsValue[4])*int_max);

			//Button START
			if(buttonsValue[7] == 1)
			{
				macro = start;
			}

			//Button BACK
			if(buttonsValue[6] == 1)
			{
				mode = static_cast<Mode>((mode + 1) % 2);
			}
			
			//BUTTON LOGITECH
			//NOTHING TO DO HERE

			//Button LJoystick && Button RJoystick
			if(buttonsValue[9] == 1 && buttonsValue[10] == 1)
			{
				macro = shutdown;
			}
		}
		
		//Buttons auf gedrückt halten überprüfen
		for(int i = 0; i < 11; i++)
		{
			if(buttonsValue[i] == 1)
			{
				buttonsPressed[i]++;
			}
			else
			{
				buttonsPressed[i] = 0;
			}
		} 

		//Orientate-Macros
		if(buttonsPressed[0] >= 15)
		{
			macro = orientate_shift;
			if(buttonsPressed[0] >= 25)
			{
				buttonsPressed[0] = 0;
				macro = DEFAULT_MACRO;
			}		
		}
		else if(buttonsPressed[1] >= 15)
		{
			macro = orientate_roll_pitch;
			if(buttonsPressed[1] >= 25)
			{
				buttonsPressed[1] = 0;
				macro = DEFAULT_MACRO;
			}		
		}
		else if(buttonsPressed[2] >= 15)
		{
			macro = orientate_level;
			if(buttonsPressed[2] >= 25)
			{
				buttonsPressed[2] = 0;
				macro = DEFAULT_MACRO;
			}
		}
		else if(buttonsPressed[3] >= 15)
		{
			macro = orientate_yaw;
			if(buttonsPressed[3] >= 25)
			{
				buttonsPressed[3] = 0;
				macro = DEFAULT_MACRO;
			}
		}

		//Commands auf die Message posten
		std::cout << "commands:{" << std::endl;
		for(int i = 0; i < 9; i++)
		{
			msg.commands.push_back((int)commands[i]);
			std::cout << commands[i] << std::endl;
		}
		std::cout << "}" << std::endl;

		//Walkingmode auf die Message posten
		switch(wmode)
		{
			case tripod:
				msg.walking_mode = "tripod";
				std::cout << "tripod" << std::endl;
				break;
			case wave:
				msg.walking_mode = "wave";
				std::cout << "wave" << std::endl;
				break;
			case ripple:
				msg.walking_mode = "ripple";
				std::cout << "ripple" << std::endl;
				break;
		}
		
		//Macro auf die Message posten
		switch(macro)
		{
			case feet:
				msg.macro = "feet";
				break;
			case orientate_shift:
				msg.macro = "orientate_shift";
				break;
			case orientate_level:
				msg.macro = "orientate_level";
				break;
			case orientate_roll_pitch:
				msg.macro = "orientate_roll_pitch";
				break;
			case orientate_yaw:
				msg.macro = "orientate_yaw";
				break;
			case start:
				msg.macro = "start";
				break;
			case shutdown:
				msg.macro = "shutdown";
				break;
			case DEFAULT_MACRO:
				msg.macro = "DEFAULT";
				break;
		}

		//publish + repeat
      movPub.publish(msg);	
      ros::spinOnce();
	   loop_rate.sleep();
   }
   return 0;
}
