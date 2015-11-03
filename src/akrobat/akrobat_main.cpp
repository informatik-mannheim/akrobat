#include <ros/ros.h>

#include <akrobat/akrobat_init.h>
#include <akrobat/Globals.h>
#include <akrobat/Akrobat.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
	init(argc, argv, "akrobat_main");

	Akrobat akrobat;
	akrobat.initAkrobat();

	// needs to be called after(below) akrobat creation (requires a ros::NodeHandle)
	Rate r_schleife(20);

	// ros main loop
	while(ok() && ON)
	{
		akrobat.runAkrobat();
		spinOnce();
		r_schleife.sleep();
	}
}