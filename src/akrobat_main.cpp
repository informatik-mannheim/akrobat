#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <akrobat/akrobat_init.h>

using namespace std;

//---------------------------------MAIN-----------------------------------//
int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "akrobat_main");
  Akrobat akrobat1;
  ros::Rate r_schleife(20);
  akrobat1.initAkrobat();

  //WHILE-LOOP
  while(ros::ok() && ON ){
	akrobat1.runAkrobat();
	ros::spinOnce();
	r_schleife.sleep();
  }//END WHILE
}//-----------------------------END Main---------------------------------//


  




    

