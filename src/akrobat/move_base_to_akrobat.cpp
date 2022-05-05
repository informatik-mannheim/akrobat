#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "akrobat/movement.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class Translater
{   
    public:
    void translate_linear(geometry_msgs::Twist msg)
	{
		std::cout<<msg<<std::endl;
	}
};



int main(int argc, char **argv)
{
   ros::init(argc, argv, "Navigation_Transllation");
   ros::NodeHandle n;
   Translater t;	
   ros::Subscriber movSub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &Translater::translate_linear, &t);
   ros::Publisher movPub = n.advertise<akrobat::movement>("movements", 1000);
   ros::Rate loop_rate(5);
}	