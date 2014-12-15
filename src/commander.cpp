#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>


ros::Subscriber js;
ros::Publisher pub_m11;
ros::Publisher pub_m12;
ros::Publisher pub_m13;

std::vector<float> old_vals;



void jointstates_callback(const sensor_msgs::JointStatePtr data)
{
   int size = data->name.size();
   std::vector<std_msgs::Float64> angles;

   std_msgs::Float64 angle;
   for(int i = 0; i < size; i++)
   {
       angle.data = data->position[i];
       angles.push_back(angle);

   }

   //did any of the joints change?
   if(fabs(old_vals[0] - angles[0].data) > 0.01 ||
           fabs(old_vals[1] - angles[1].data)  > 0.01 ||
           fabs(old_vals[2] - angles[2].data)  > 0.01)
   {
       pub_m11.publish(angles[0]);
       pub_m12.publish(angles[1]);
       pub_m13.publish(angles[2]);

       old_vals[0] = angles[0].data;
       old_vals[1] = angles[1].data;
       old_vals[2] = angles[2].data;

   }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"commander");
    ros::NodeHandle nh;

    old_vals.resize(3);
    old_vals[0] = 0.0;
    old_vals[1] = 0.0;
    old_vals[2] = 0.0;

    js = nh.subscribe("/joint_commands",1,jointstates_callback);
    pub_m11 = nh.advertise<std_msgs::Float64>("/m11/command",1);
    pub_m12 = nh.advertise<std_msgs::Float64>("/m12/command",1);
    pub_m13 = nh.advertise<std_msgs::Float64>("/m13/command",1);





    ros::spin();
	return	0;
}
