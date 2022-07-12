#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "akrobat/movement.h"
#include "akrobat/Translater.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


float x;
void translate_linear(geometry_msgs::Twist msg)
	{
        x = msg.linear.x;

		ROS_ERROR("%g",x);
	}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "Navigation_Transllation");
   ros::NodeHandle n;
   Translater t;	
   ros::Subscriber movSub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, translate_linear);
   //ros::Publisher movPub = n.advertise<akrobat::movement>("joy", 1000);
   ros::Rate loop_rate(5);

   while (ros::ok())
	{
		ros::spinOnce();
        
		loop_rate.sleep();
	}
}	