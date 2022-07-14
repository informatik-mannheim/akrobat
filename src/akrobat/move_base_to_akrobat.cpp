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
      
      pad.axes[1]=x*5;
      pad.axes[0]=y*5;

      if (z>0)
      {
         z = z*2;
         pad.axes[2]=1-z;
         pad.axes[5]=1;

      }
      
      if (z<0)
      {
         z = z*2;
         pad.axes[5]=1-z;
         pad.axes[2]=1;

      }
      ROS_ERROR("X:%g",x);
      ROS_ERROR("Y:%g",y);
      ROS_ERROR("Z:%g",z);
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
   movPub = n.advertise<sensor_msgs::Joy>("joy_auto", 1000);
   ros::Rate loop_rate(5);
   pad.axes.resize(8);
   pad.buttons.resize(11);
   while (ros::ok())
	{  
      
		ros::spinOnce();
        
		loop_rate.sleep();
	}
}	