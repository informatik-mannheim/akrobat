/** @file akrobat_main.cpp
 *  @brief Akrobat main file with ROS loop.
 *
 *  @author Author
 */

#include <ros/ros.h>
#include <akrobat/Akrobat.h>

using namespace std;
using namespace ros;

/** Main method with ROS main loop.
*
*   @return Integer.
*/
int main(int argc, char** argv)
{
	init(argc, argv, "akrobat_main");

	Akrobat akrobat;

	akrobat.initAkrobat();
	
	// needs to be called after(below) akrobat creation (requires a ros::NodeHandle)
	Rate spinRate(20);

	// ros main loop
	while (ok())
	{
		akrobat.runAkrobat();
		spinOnce();
		spinRate.sleep();
	}
}
