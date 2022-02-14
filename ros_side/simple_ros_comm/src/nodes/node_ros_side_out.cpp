#include <ros_side_out.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ROSSideOutNode");
	boost::asio::io_context io_context;
	ros::NodeHandle n;

	std::string packpath = ros::package::getPath("simple_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_ros.yaml");

	struct ROSSideOutConfig cfg;
	cfg.port_out = f["PORT_OUT"].as<int>();
	cfg.msg_size = f["MSG_SIZE"].as<int>();
	cfg.subscriber_name = f["SUBSCRIBER_NAME"].as<std::string>();
	cfg.verbose = f["VERBOSE"].as<int>();

	ROSSideOut server(n, io_context, cfg);

	ros::spin();


	return 0;
}