/* @file Dynamixel_Controlle.cpp
 *  @brief Dynamixel Controller and Writer with ROS loop.
 *
 *  @author Author
 */



// Code from Tutorial SDK CPP Sync Write Protocol 1.0
// URL https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/cpp_sync_write_protocol_1_0/#cpp-sync-write-protocol-10

#include "dynamixel_motor/Dynamixel_Controll.h"

// Uses DYNAMIXEL SDK library
// Controll table: https://emanual.robotis.com/docs/en/dxl/rx/rx-64/#control-table

// Control table address
#define ADDR_RX_TORQUE_ENABLE			24                  
#define ADDR_RX_Moving_Speed			32
#define ADDR_RX_GOAL_POSITION			30
#define ADDR_RX_PRESENT_POSITION		36
#define ADDR_RX_PRESENT_Load			40
#define ADDR_RX_Delay_Time				5 

// Data Byte Length
#define LEN_RX_Moving_Speed         	2
#define LEN_RX_Delay_Time				1
#define LEN_RX_GOAL_POSITION			2
#define LEN_RX_PRESENT_POSITION			2
#define LEN_RX_PRESENT_Load				2

// Protocol version
#define PROTOCOL_VERSION				1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE						1000000
#define DEVICENAME						"/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10               SubscribeAndPublish SAPObject;   // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


using namespace std;

ros::Publisher dynamixel_status; 

/** Get Info from the ROS parameters.
 * 
* @return bool
*/

bool DynamixelController::get_info()
{
	std::vector<std::string> motor;
	node_handle_.getParam("/akrobat_config/motoren",motor);
	
	int i;
	for (i=0;i<motor.size();i++)
	{
		std::string name;
		name = motor[i];
		node_handle_.getParam("/akrobat_config/"+name+"/ID", id);
		node_handle_.getParam("/akrobat_config/"+name+"/Moving_Speed", move);
		node_handle_.getParam("/akrobat_config/"+name+"/Return_Delay_Time", delay);
	
		ItemValue item_value = {id, move,delay};
		std::pair<std::string, ItemValue> motor_info(name, item_value);
		dynamixel_info_.push_back(motor_info);
	}

	return true;
}

/** Initzialize the Dynamixel controller.
 * 
* @return bool
*/
bool DynamixelController::controler_initialize()
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort())
	{
	  	printf("Succeeded to open the port!\n");
	}
	else
	{
	  	printf("Failed to open the port!\n");
	  	printf("Press any key to terminate...\n");
	  	return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
	  	printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
	  	printf("Press any key to terminate...\n");
	  	return 0;
	}


	return true;
}

/** Initzialize Dynamixel motors
 * 
* @return bool
*/

bool DynamixelController::motor_initialize()
{
 	
 	dynamixel::GroupSyncWrite move_speed_sync_write(portHandler, packetHandler, ADDR_RX_Moving_Speed, LEN_RX_Moving_Speed);
 	dynamixel::GroupSyncWrite delay_time_sync_write(portHandler, packetHandler, ADDR_RX_Delay_Time, LEN_RX_Delay_Time);
 	
 	
	for(m = 0;m<18;m++)
	{
	   	//Data read for Motor
		motor_info = dynamixel_info_[m];	
	  
		ID = motor_info.second.id;
	    	
		//Motor Initzialisierung
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	    
		if (dxl_comm_result != COMM_SUCCESS)
		{
			ROS_ERROR(" Intizial Error: %s",packetHandler->getTxRxResult(dxl_comm_result));
			
		}
		else if (dxl_error != 0)
		{
			ROS_ERROR("Intizial Error: %s",packetHandler->getRxPacketError(dxl_error));
			
		}
		else
		{
			ROS_INFO("Dynamixel#%d has been successfully connected", ID);
		}
	    
	
	
		//Setting Moving Speed
		int move_speed;
		move_speed = dynamixel_info_[m].second.move;
		param_move_speed[0] = DXL_LOBYTE(move_speed);
		param_move_speed[1] = DXL_HIBYTE(move_speed);
		
		dxl_addparam_result = move_speed_sync_write.addParam(ID, param_move_speed);
			
				
		if (dxl_addparam_result != true)
		{
			ROS_ERROR("[ID:%d] move_speed_sync_write addparam failed", ID);	
		}
		
		
		//Setting Return_Delay_Time
		int delay_time;
		delay_time = dynamixel_info_[m].second.delay;
		param_delay_time[0] = DXL_LOBYTE(delay_time);
		param_delay_time[1] = DXL_HIBYTE(delay_time);
		dxl_addparam_result = delay_time_sync_write.addParam(ID, param_delay_time);
		
		
	}
	
	dxl_comm_result = move_speed_sync_write.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
	}
	
	dxl_comm_result = delay_time_sync_write.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
	}
	 
	return true; 
}

/** Subscriber for the goal position from Akrobat
 * 
* @return bool
*/

bool DynamixelController::sub_status()
{
	dyn_status = node_handle_.subscribe("/dynamixel_status",10,&DynamixelController::chatterCallback,this);

	return true;
}

/** Subscriber for shutdown from Akrobat
 * 
* @return bool
*/


void DynamixelController::chatterCallback(const dynamixel_motor::dynamixel & msg)
{

}

bool DynamixelController::sub_positions()
{
	node_handle_.param<std::string>("goalNodeName", goalNodeName, "/goal_joint_states");
	goal_joint_states = node_handle_.subscribe(goalNodeName,10,&DynamixelController::position,this);

	return true;
}


/** Sync write goal position to Dynamixel Motors
 * 
* @return void
*/

void DynamixelController::position(const sensor_msgs::JointState::ConstPtr& msg)
{
	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_RX_GOAL_POSITION, LEN_RX_GOAL_POSITION);
	
	// reading position from Akrobat and ID from config
	for (int i = 0; i < msg->name.size(); i++)
	{
		ID=dynamixel_info_[i].second.id;			//ID aus Config
		pos_rad = msg->position[i];
		float pos_end;
		//Kalkulation for Dynamixel
		pos_end = (150 + pos_rad *(180/3.14159265358979323846))*(512.0/150);
		
		// Allocate goal position value into byte array
		param_goal_position[0] = DXL_LOBYTE(pos_end);
		param_goal_position[1] = DXL_HIBYTE(pos_end);
			
		dxl_addparam_result = groupSyncWrite.addParam(ID, param_goal_position);
		
		if (dxl_addparam_result != true)
		{
			ROS_ERROR("[ID:%d] groupSyncWrite addparam failed", ID);	
		}	
	}

	
	
	// Catch Error
	dxl_comm_result = groupSyncWrite.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
	}
	// Clear syncwrite parameter storage
	groupSyncWrite.clearParam();

	DynamixelController::cur_position();
	
}

bool DynamixelController::cur_position()	
{
	
	
	

	dynamixel_msg.name.resize(18);
	dynamixel_msg.position.resize(18);
	dynamixel_msg.velocity.resize(18);
	dynamixel_msg.effort.resize(18);
	dynamixel_msg.load.resize(18);

	
	node_handle_.getParam("/akrobat_config/motoren",motor);
	
	int i;

	for (i=0;i<motor.size();i++)
	{
		std::string name;
		name = motor[i];
		node_handle_.getParam("/akrobat_config/"+name+"/ID", id);

		// // Get Dynamixel present position valuestring dyn_name = ;
		dxl_read_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_RX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
		if (dxl_read_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_read_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}

		// Getiing Dynamixel present load
		dxl_read_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_RX_PRESENT_Load, &dxl1_present_load, &dxl_error);
		if (dxl_read_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_read_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		
	
		dynamixel_msg.name[i] = "m"+to_string(id);
		dynamixel_msg.velocity[i] = 0.0;
		dynamixel_msg.effort[i] = 0.0;
		dynamixel_msg.load[i] = dxl1_present_load;
		dynamixel_msg.position[i]= dxl1_present_position;
		dynamixel_msg.header.stamp = ros::Time::now();
		
		
	}
	dynamixel_status.publish(dynamixel_msg);
	
	
	return true;
	
}

bool DynamixelController::sub_down()
{	
	
	mov_status = node_handle_.subscribe("/movements",1,&DynamixelController::torqueoff,this);

	return true;
}

void DynamixelController::torqueoff(const akrobat::movement::ConstPtr& msg)
{	
	
if (msg->macro == "shutdown")
	{	
		ROS_ERROR("Shutdown");
		node_handle_.getParam("/akrobat_config/motoren",motor);
		
		int i;

		for (i=0;i<motor.size();i++)
		{	
			std::string name;
			name = motor[i];
			node_handle_.getParam("/akrobat_config/"+name+"/ID", id);

			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_RX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

			
		}
		// Close port
		portHandler->closePort();
		
	}
	
}


/** Main
 * 
* @return int
*/

int main(int argc, char *argv[])
{
	
	
	ros::init(argc, argv, "Dynamixel");
	ros::NodeHandle n;
	dynamixel_status = n.advertise<dynamixel_motor::dynamixel>("/dynamixel_status", 1);
	DynamixelController dynamixel_controller;

	int publish_frequency = 10;
	bool result = false;

	ros::Rate spinRate(publish_frequency);
	
	std::string config;
	n.getParam("dynamixel_info" , config);
  
	// Get Info from Parameter File
	result = dynamixel_controller.get_info(); 
	if (result == false)
		ROS_ERROR("Config Data not found");

	// Initzialize Dynamixel Controller
	result = dynamixel_controller.controler_initialize();
	if (result == false)
		ROS_ERROR("Initalize Controller failed");
	
	// Initzialize Dynamixel Motor
	result = dynamixel_controller.motor_initialize();
	if (result == false)
		ROS_ERROR("Initalize Motor failed");

	// Subscriber for Position Data and write position to Motor
	result = dynamixel_controller.sub_positions();
	if (result == false)
		ROS_ERROR("Subscriber Error");
    //Subscriber for Shutdown
	result = dynamixel_controller.sub_down();
	if (result == false)
		ROS_ERROR("Shutdown Error");
	

	// ROS main loop
	while (ros::ok())
	{
		ros::spinOnce();
		
		spinRate.sleep();
		
	}
	
	
	

 }