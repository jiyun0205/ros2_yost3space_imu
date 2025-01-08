#include <Y3SpaceDriver.h>


// int main(int argc, char **argv)
// {
//   	ros::init(argc, argv, "Y3SpaceDriver");
//   	ros::NodeHandle nh;
//   	ros::NodeHandle pnh("~");

//   	std::string port;
//   	int baudrate;
//   	int timeout;
//   	std::string mode;
//   	std::string frame;

//     pnh.param<std::string>("port", port, "/dev/ttyIMU"); 
//     pnh.param<int>("baudrate", baudrate, 115200);
//     pnh.param<int>("timeout", timeout, 3000);  // 60000
//     pnh.param<std::string>("mode", mode, "relative");
//     pnh.param<std::string>("frame", frame, "imu_link");

//   	Y3SpaceDriver driver(nh, pnh, port, baudrate, timeout, mode, frame);
//   	driver.run();
// }
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "Y3SpaceDriver.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::NodeOptions options;

	// std::string port;
    // int baudrate;
    // int timeout;
    // std::string mode;
    // std::string frame;
	
    // auto node = std::make_shared<rclcpp::Node>("Y3SpaceDriver", options);

    // node->declare_parameter<std::string>("port", "/dev/imu");
    // node->declare_parameter<int>("baudrate", 115200);
    // node->declare_parameter<int>("timeout", 3000);  // 60000
    // node->declare_parameter<std::string>("mode", "relative");
    // node->declare_parameter<std::string>("frame", "imu_link");

    // node->get_parameter("port", port);
    // node->get_parameter("baudrate", baudrate);
    // node->get_parameter("timeout", timeout);
    // node->get_parameter("mode", mode);
    // node->get_parameter("frame", frame);

    // auto driver = std::make_shared<Y3SpaceDriver>(port, baudrate, timeout, mode, frame);
	auto driver = std::make_shared<Y3SpaceDriver>();

    driver->run();
    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char * argv[])
// {
// 	rclcpp::init(argc, argv);
// 	rclcpp::NodeOptions options;
// 	// auto driver = std::make_shared<Y3SpaceDriver>(options);

// 	std::string port;
//   	int baudrate;
//   	int timeout;
//   	std::string mode;
//   	std::string frame;



//     pnh.param<std::string>("port", port, "/dev/imu"); 
//     pnh.param<int>("baudrate", baudrate, 115200);
//     pnh.param<int>("timeout", timeout, 3000);  // 60000
//     pnh.param<std::string>("mode", mode, "relative");
//     pnh.param<std::string>("frame", frame, "imu_link");


//     auto driver = std::make_shared<Y3SpaceDriver>(port, baudrate, timeout, mode, frame);

// 	driver->run();
// 	rclcpp::shutdown();
// 	return 0;
// 	}