#ifndef RUMBLEPAD2STRUCT_H
#define RUMBLEPAD2STRUCT_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

//joypad
struct rumblePad2Struct
{
	tf::Vector3 speed; // forward/backward/sideward movement
	tf::Vector3 bdR;   // body rotation
	tf::Vector3 bdT;   // body translation
};

#endif // RUMBLEPAD2STRUCT_H