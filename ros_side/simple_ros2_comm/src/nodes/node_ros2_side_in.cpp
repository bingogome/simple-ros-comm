#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <ros2_side_in.hpp>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	boost::asio::io_context io_context;

	std::string packpath = ament_index_cpp::get_package_share_directory("simple_ros2_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/../../../../src/simple_ros2_comm/config_ros2.yaml");

	struct ROS2SideInConfig cfg;
	cfg.port_in = f["PORT_IN"].as<int>();
	cfg.msg_size = f["MSG_SIZE"].as<int>();
	cfg.end_msg = f["MSG_END"].as<std::string>();
	cfg.publisher_name = f["PUBLISHER_NAME"].as<std::string>();
	cfg.verbose = f["VERBOSE"].as<int>();

	try
	{
		ROS2SideIn server(io_context, cfg);
		io_context.run(); 
		// For my own note: 
		// use io_context.poll() if do not want it run indefinitely
	}
	catch (std::exception& e)
	{
		io_context.stop();
        throw;
	}
	return 0;
}