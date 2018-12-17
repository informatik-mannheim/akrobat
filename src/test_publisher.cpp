#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <vector>
#include "akrobat_hector/movement.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

int int_max = std::numeric_limits<int>::max();

enum Mode 
{
	navigate, 
	work
};
enum Walking_Mode 
{
	tripod, 
	wave, 
	ripple
};
enum Mode_right_joystick 
{
	DEFAULT_MODE, 
	shift, 
	roll_pitch, 
	yaw, 
	level
};
enum Macro 
{
	DEFAULT_MACRO, 
	feet
};

std::vector<float> axis (0);
std::vector<int> buttons (0);
std::string controller_type = "DEFAULT!";
int amountAxis = -1;
int amountButtons = -1;

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
		controller_type = "Logitech Gamepad F710";
	}
   for(int i = 0; i < amountAxis; i++)
   {
      axis[i] = msg->axes[i];
   }
   for(int i = 0; i < amountButtons; i++)
   {
      buttons[i] = msg->buttons[i];
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "testPublisher");
   ros::NodeHandle n;
   ros::Subscriber testSubscriber = n.subscribe<sensor_msgs::Joy>("joy", 1000, readJoypadCallback);
   ros::Publisher testPub = n.advertise<akrobat_hector::movement>("testMovements", 1000);
   ros::Rate loop_rate(10);
   while(ros::ok())
   {
      akrobat_hector::movement msg;
		
		/*if(controller_type != "Logitech Gamepad F710")
		{
			continue;
		}*/

		int commands[9] = {0}; //w_x, w_y, w_alpha, yaw, pitch, roll, m_x, m_y, m_z
		Mode mode = navigate;
		Walking_Mode wmode = tripod;
		Macro macro = DEFAULT_MACRO;
		Mode_right_joystick m = DEFAULT_MODE;
		
		//linker Joystick
		commands[0] = axis[1]*int_max;
		commands[1] = axis[0]*int_max;

		//rechter Joystick
		if(m != DEFAULT_MODE)
		{
			if(m == shift)
			{
				commands[6] = axis[3]*int_max;
				commands[7] = axis[2]*int_max;
			}
			else if(m == roll_pitch)
			{
				commands[4] = axis[3]*int_max;
				commands[5] = axis[2]*int_max;
			}
			else if(m == yaw)
			{
				commands[3] = axis[2]*int_max;
			}
			else if(m == level)
			{
				commands[8] = axis[3]*int_max;
			}
		}

		//RT&LT
		commands[2] = ((axis[4]-axis[5])*int_max)/2;

		//Dig. Joystick
		if(axis[7] > 0)
		{
			macro = feet;
		}
		else
		{
			macro = DEFAULT_MACRO;
		}
		
		//Button A
		if(buttons[0] == 1)
		{
			m = level;
		}

		//Button B
		if(buttons[1] == 1)
		{
			m = yaw;
		}

		//Button X
		if(buttons[2] == 1)
		{
			m = shift;
		}

		//Button Y
		if(buttons[3] == 1)
		{
			m = roll_pitch;
		}

		//Button LB
		if(buttons[4] == 1)
		{
			wmode = static_cast<Walking_Mode>((wmode + 1) % 3);
		}

		//Button RB
		if(buttons[5] == 1)
		{
			mode = static_cast<Mode>((mode + 1) % 2);
		}

		//Button START
		//NOTHING TO DO HERE

		//Button BACK
		//NOTHING TO DO HERE

		//Button Logitech
		//NOTHING TO DO HERE

		//Button LJoystick
		//NOTHING TO DO HERE

		//Button RJoystick
		//NOTHING TO DO HERE
      
      /*for(int i = 0; i < 10 && ros::ok(); i++)
      {
         std::string s[] = {"w_x", "w_y", "w_alpha", "yaw", "pitch", "roll", "m_x", "m_y", "m_z", "mode"};
         std::string input = "";
         std::cout << "Geben Sie einen Wert für " << s[i] << " an" << std::endl;
         std::cin >> input;
         int val = 0;
         std::string sval = "";
         if(s[i] != "mode")
         {
            val = atoi(input.c_str());
         }
         else
         {
            sval = input;
         }
         
         switch(i)
         {
            case 0:
               msg.w_x = val;
               break;
            case 1:
               msg.w_y = val;
               break;
            case 2:
               msg.w_alpha = val;
               break;   
            case 3:
               msg.yaw = val;
               break;
            case 4:
               msg.pitch = val;
               break;
            case 5:
               msg.roll = val;
               break;
            case 6:
               msg.m_x = val;
               break;
            case 7:
               msg.m_y = val;
               break;
            case 8:
               msg.m_z = val;
               break;
            case 9:
               msg.mode = input;
               break;
         }
      }
      std::cout << amountButtons << std::endl;*/

		for(int i = 0; i < 9; i++)
		{
			msg.commands.push_back(commands[i]);
		}
		switch(wmode)
		{
			case tripod:
				msg.walking_mode = "tripod";
				break;
			case wave:
				msg.walking_mode = "wave";
				break;
			case ripple:
				msg.walking_mode = "ripple";
				break;
		}
		switch(macro)
		{
			case feet:
				msg.macro = "feet";
				break;
			case DEFAULT_MACRO:
				msg.macro = "DEFAULT";
				break;
		}

      testPub.publish(msg);
		std::string wait = "";
		std::cout << "Hallo Welt!";
		std::cin >> wait;
      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}
