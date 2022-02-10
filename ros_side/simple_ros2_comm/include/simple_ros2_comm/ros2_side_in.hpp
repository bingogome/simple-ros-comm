#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

// Configuration parameters struct. Will be initialized by 
// data from config_ros2.yaml
struct ROS2SideInConfig
{
	int port_in;
	std::string end_msg;
	std::string publisher_name;
	int verbose;
	int msg_size;
};

// A publisher class that contains the reference to a node.
// Will be used to spin in a separated std::thread from the main
// thread.
class MinimalPublisher
{

public:
	MinimalPublisher(struct ROS2SideInConfig cfg);
	void PublishThis(std_msgs::msg::String msg_test);
	void SpinROS();
	void RunThread();
	rclcpp::Logger GetLogger();

private:

	struct ROS2SideInConfig cfg_;

	// ros related members
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	
};

// Main class of the program. Will be called by the main function and constantly
// receives messages
// Contains a MinimalPublisher object to be spun (ROS spin) in a separate std::thread
class ROS2SideIn
{

public: 
	ROS2SideIn(boost::asio::io_context& io_context, struct ROS2SideInConfig cfg);

private:
	void StartReceive();
	void EndServerClean();
	void HandleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void HandleIncoming();

	std_msgs::msg::String msg_test_;
	std::string sscmd_str_; // command header of current messege
	std::string ss_str_; // the whole msg
	struct ROS2SideInConfig cfg_;
	MinimalPublisher pub_;

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 55> recv_buffer_; // TODO: make this configurable 55 chars in an encoded command
	// TODO: make socket array so that multiple channels are possible (or dynamic expansion)
	// TODO: make TCP optional
	
};