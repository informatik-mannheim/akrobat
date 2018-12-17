#include <ros/ros.h>

#include <controller_manager/controller_manager.h>

#include <akrobat/HardwareInterface.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
	init(argc, argv, "Controller");

	Akrobat::HardwareInterface hardwareInterface;
	controller_manager::ControllerManager controllerManager(&hardwareInterface);

	Rate spinRate(20);

	while (ok())
	{
		//hardwareInterface.read();
		controllerManager.update(hardwareInterface.getTime(), hardwareInterface.getPeriod());
		hardwareInterface.write();
		spinOnce();
		spinRate.sleep();
	}
}