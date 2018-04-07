#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "akrobat_hector/movement.h"

void testCallback(const akrobat_hector::movement::ConstPtr& msg)
{
   std::cout << msg->walking_mode.c_str() << std::endl;
}

void testJsCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
   std::cout << msg->buttons[0] << msg->axes[0] << std::endl;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_subscriber");
   ros::NodeHandle n;
   ros::Subscriber testSub = n.subscribe("testMovements", 1000, testCallback);
   ros::Subscriber testSub2 = n.subscribe("joy", 1000, testJsCallback);
   ros::spin();
}
