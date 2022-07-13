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
float y;
float z;

sensor_msgs::Joy pad;
ros::Publisher movPub;

void translate_linear(geometry_msgs::Twist msg)
	{
      x = msg.linear.x;
      y = msg.linear.y;
      z = msg.angular.z;
      
      pad.axes[0]=x;
      pad.axes[1]=y;

      if (z>0)
      {
         z = z*2;
         pad.axes[2]=1-z;

      }
      
      if (z<0)
      {
         z = z*2;
         pad.axes[5]=1-z;

      }
      ROS_ERROR("%g",pad.axes[1]);
      pad.header.stamp = ros::Time::now();
      movPub.publish(pad);
      ros::Rate rate(5);
      rate.sleep();

		
	}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "Navigation_Transllation");
   ros::NodeHandle n;
   Translater t;	
   ros::Subscriber movSub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, translate_linear);
   movPub = n.advertise<sensor_msgs::Joy>("joy_auto", 10);
   ros::Rate loop_rate(5);
   pad.axes.resize(8);
   while (ros::ok())
	{  
      
		ros::spinOnce();
        
		loop_rate.sleep();
	}
}	