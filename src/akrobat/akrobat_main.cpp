#include <ros/ros.h>
#include <akrobat/akrobat_init.h>
#include <akrobat/Akrobat.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
	init(argc, argv, "akrobat_main");
	Rate r_schleife(20);

	Akrobat akrobat;
	akrobat.initAkrobat();

	// ros main loop
	while (ok() && ON) {
		akrobat.runAkrobat();
		spinOnce();
		r_schleife.sleep();
	}
}