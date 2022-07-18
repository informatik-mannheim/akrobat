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
      x = msg.linear.x*12;
      y = msg.linear.y*12;
      z = msg.angular.z*3;

      if(x>1)
      {
         x = 1;
      }

      if(y>1)
      {
         y = 1;
      }

      if(z>1)
      {
         z=0.5;
      }
      

      pad.axes[1]=x;       //Vorwärts
      pad.axes[0]=y;       //Seitwärts
      pad.axes[2]=z;       //Drehung

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