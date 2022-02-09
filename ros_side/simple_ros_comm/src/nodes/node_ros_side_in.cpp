#include <ros/ros.h>
#include <ros_side_in.hpp>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ROSSideInNode");
	boost::asio::io_context io_context;
	ros::NodeHandle n;

	std::string packpath = ros::package::getPath("simple_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_ros.yaml");

	struct ROSSideInConfig cfg;
	cfg.port_in = f["PORT_IN"].as<int>();
	cfg.msg_size = f["MSG_SIZE"].as<int>();
	cfg.end_msg = f["MSG_END"].as<std::string>();
	cfg.publisher_name = f["PUBLISHER_NAME"].as<std::string>();
	cfg.verbose = f["VERBOSE"].as<int>();

	try
	{
		ROSSideIn server(n, io_context, cfg);
		io_context.run(); 
		// For my own note: 
		// use io_context.poll() if do not want it run indefinitely
	}
	catch (std::exception& e)
	{
		io_context.stop();
		ROS_INFO("Error happened, or user interrupted");
        throw;
	}

	return 0;
}