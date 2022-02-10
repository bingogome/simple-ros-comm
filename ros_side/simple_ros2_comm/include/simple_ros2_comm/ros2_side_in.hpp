#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

struct ROS2SideInConfig
{
	int port_in;
	std::string end_msg;
	std::string publisher_name;
	int verbose;
	int msg_size;
};

class MinimalPublisher
{

public:
	MinimalPublisher(struct ROS2SideInConfig cfg);
	void PublishThis(std_msgs::msg::String msg_test);
	void RunThread();
	rclcpp::Logger GetLogger();

private:

	struct ROS2SideInConfig cfg_;

	// ros related members
	rclcpp::Node node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	
};

class ROS2SideIn
{

public: 
	ROS2SideIn(boost::asio::io_context& io_context, struct ROS2SideInConfig cfg);

private:
	void StartReceive();
	void EndServerClean();
	void HandleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void HandleIncoming();

	// asio related members
	std_msgs::msg::String msg_test_;
	std::string sscmd_str_; // command header of current messege
	std::string ss_str_; // the whole msg
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 55> recv_buffer_; // TODO: make this configurable 55 chars in an encoded command
	// TODO: make socket array so that multiple channels are possible (or dynamic expansion)
	// TODO: make TCP optional

	struct ROS2SideInConfig cfg_;
	MinimalPublisher pub_;
	
};