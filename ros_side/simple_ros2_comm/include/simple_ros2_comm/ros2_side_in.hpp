#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct ROS2SideInConfig
{
	int port_in;
	std::string end_msg;
	std::string publisher_name;
	int verbose;
	int msg_size;
};

class MinimalPublisher : public rclcpp::Node
{

public:
	MinimalPublisher();

private:
	// ros related members
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	std_msgs::String msg_test_;
	std::string sscmd_str_; // command header of current messege
	std::string ss_str_; // whole msg
};

class ROS2SideIn
{

public: 
	ROS2SideIn(boost::asio::io_context& io_context, struct ROSSideInConfig cfg);

private:
	void StartReceive();
	void EndServerClean();
	void HandleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void HandleIncoming();

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 55> recv_buffer_; // TODO: make this configurable 55 chars in an encoded command
	// TODO: make socket array so that multiple channels are possible (or dynamic expansion)
	// TODO: make TCP optional

	struct ROSSideInConfig cfg_;
	MinimalPublisher pub_;
	
};