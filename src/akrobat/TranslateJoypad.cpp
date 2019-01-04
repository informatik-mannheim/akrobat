#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "akrobat/movement.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

using namespace std;

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
	vector<float> getAxisValue()
	{
		return axis;
	}

	void setAxisValue(vector<float> av)
	{
		axis = av;
	}

	vector<int> getButtonsValue()
	{
		return buttons;
	}

	void setButtonsValue(vector<int> bv)
	{
		buttons = bv;
	}

	string getControllerType()
	{
		return controller_type;
	}

	void setControllerType(string ct)
	{
		controller_type = ct;
	}

	void readJoypadCallback(const sensor_msgs::Joy::ConstPtr& msg)
	{
		amountAxis = msg->axes.size();
   	    amountButtons = msg->buttons.size();
		cout << "Anzahl Achsen: " << amountAxis << "\n Anzahl Buttons:" << amountButtons << endl;
		if(amountAxis == 8 && amountButtons == 11)
		{
			/*
				Achsennummern:						Buttonnummern:
				0: Joystick links, rechts<->links	0: A
				1: Joystick links, oben<->unten		1: B
				2: Joystick rechts, rechts<->links	2: X
				3: Joystick rechts, oben<->unten	3: Y
				4: LT								4: LB
				5: RT								5: RB
				6: Joystick, rechts<->links	        6: BACK
				7: Joystick, oben<->unten		    7: START
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
	vector<float> axis;
	vector<int> buttons;
	string controller_type = "DEFAULT!";
	int amountAxis = -1;
	int amountButtons = -1;
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "translateJoypad");
   ros::NodeHandle nh;
   Listener l;
   ros::Subscriber joySub = nh.subscribe<sensor_msgs::Joy>("joy", 1000, &Listener::readJoypadCallback, &l);
   ros::Publisher movPub = nh.advertise<akrobat::movement>("movements", 1000);
   ros::Rate loop_rate(5);
	Mode mode = navigate;
	Walking_Mode wmode = tripod;
	Mode_right_joystick m = DEFAULT_MODE;
	Macro macro = start;
	vector<int> buttonsPressed = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

   while(ros::ok())
   {
		vector<float> axisValue = l.getAxisValue();
		vector<int> buttonsValue = l.getButtonsValue();
		string ct = l.getControllerType();
        akrobat::movement msg;
        akrobat::movement msg;

		//Überprüfen, ob der richtige Controller angeschlossen ist
		if(ct != "Logitech Gamepad F710")
		{
			cout << "Falscher Controller" << endl;
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		//w_x, w_y, w_alpha, yaw, pitch, roll, m_x, m_y, m_z
        Map<string, float> commands;

		//Befehle im Navigationsmodus
		if(mode == navigate)
		{
			//LINKER JOYSTICK
			if(abs(axisValue[1]) > joystickDeadZone)
			{ 
				commands["w_x"] = axisValue[1] * int_max;
			}
			else
			{
			
			commandss["w_x"] = 0;
			if(abs(axisValue[0]) > joystickDeadZone)
			{
				commands["w_y"] = axisValue[0]*int_max*(-1);
			}
			else
			{
				commands["w_y"] = 0;
			}

			//RECHTER JOYSTICK
			if(m != DEFAULT_MODE)
			{
				if(m == shift)
				{
					if(abs(axisValue[4]) > joystickDeadZone)
					{
						commands["m_x"] = axisValue[4]*int_max;
					}
					else
					{
						commands["m_x"] = 0;
					}
					if(abs(axisValue[3]) > joystickDeadZone)
					{					
						commands["m_y"] = axisValue[3]*int_max;
					}
					else
					{
						commands["m_y"] = 0;
					}
				}
				else if(m == roll_pitch)
				{
					if(abs(axisValue[4]) > joystickDeadZone)
					{
						commands["pitch"] = axisValue[4]*int_max;
					}
					else
					{
						commands["pitch"] = 0;
					}
					if(abs(axisValue[3]) > joystickDeadZone)
					{
						commands["roll"] = axisValue[3]*int_max;
					}
					else
					{
						commands["roll"] = 0;
					}
				}
				else if(m == yaw)
				{
					if(abs(axisValue[3]) > joystickDeadZone)
					{
						commands["yaw"] = axisValue[3]*int_max;
					}
					else
					{
						commands["yaw"] = 0;
					}
				}
				else if(m == level)
				{
					if(abs(axisValue[4]) > joystickDeadZone)
					{
						commands["m_z"] = axisValue[4]*int_max;
					}
					else
					{
						commands["m_z"] = 0;
					}
				}
			}
	
			//RT&LT
			if(abs(axisValue[2] - axisValue[5]) > joystickDeadZone)
			{
				commands["w_y"] = ((axisValue[2]-axisValue[5])*int_max)/2;
				//Im Uhrzeigersinn drehen --> positiv
			}
			else
			{
				commands["w_y"] = 0;
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
			if(abs(axisValue[1]) > joystickDeadZone)
			{
				commands["m_x"] = axisValue[1]*int_max;
			}		
			else
			{
				commands["m_x"] = 0;
			}		
			if(abs(axisValue[0]) > joystickDeadZone)
			{
				commands["m_y"] = axisValue[0]*int_max*(-1);
			}
			else
			{
				commands["m_y"] = 0;
			}

			//RECHTER JOYSTICK
			if(abs(axisValue[4]) > joystickDeadZone)
			{			
				commands["pitch"] = axisValue[4]*int_max;
			}
			else
			{
				commands["pitch"] = 0;
			}
			if(abs(axisValue[3]) > joystickDeadZone)
			{			
				commands["roll"] = axisValue[3]*int_max*(-1);
			}
			else
			{
				commands["roll"] = 0;
			}

			//RT&LT
			if(abs(axisValue[2] - axisValue[5]) > joystickDeadZone)
			{
				commands["yaw"] = ((axisValue[2]-axisValue[5])*int_max)/2;
				//Im Uhrzeigersinn --> positiv
			}
			else
			{
				commands["yaw"] = 0;
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
			commands["m_z"] = ((buttonsValue[5]-buttonsValue[4])*int_max);

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
		cout << "commands:{" << endl;
		for(auto& [key, value]: commands)
        {
  		     msg.commands.push_back((int)value)
  		     cout << commands[i] << endl;
        }
		cout << "}" << endl;

		//Walkingmode auf die Message posten
		switch(wmode)
		{
			case tripod:
				msg.walking_mode = "tripod";
				cout << "tripod" << endl;
				break;
			case wave:
				msg.walking_mode = "wave";
				cout << "wave" << endl;
				break;
			case ripple:
				msg.walking_mode = "ripple";
				cout << "ripple" << endl;
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
