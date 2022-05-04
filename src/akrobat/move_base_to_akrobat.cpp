int main(int argc, char **argv)
{
   ros::init(argc, argv, "move_base_to_akrobat");
   ros::NodeHandle n;
	
   ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &Listener::readJoypadCallback, &l);
	 ros::Subscriber movSub = n.subscribe<akrobat::movement>("cmd_vel", 1000, &Listener::readMovementCallback, &l);
   ros::Publisher movPub = n.advertise<akrobat::movement>("movements", 1000);
   ros::Rate loop_rate(5);
}	