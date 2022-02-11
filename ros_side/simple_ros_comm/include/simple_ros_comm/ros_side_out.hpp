#pragma once

// Configuration parameters struct. Will be initialized by 
// data from config_ros.yaml
struct ROSSideOutConfig
{
	int port_out;
	std::string end_msg;
	std::string subscriber_name;
	int verbose;
	int msg_size;
};

class ROSSideOut
{

public:

	ROSSideOut(ros::NodeHandle& n, boost::asio::io_context& io_context, 
		struct ROSSideInConfig cfg);

private:

	// ros related members
	ros::NodeHandle& n_;
	ros::Subscriber sub_;
	std_msgs::String msg_test_;

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;

	struct ROSSideOutConfig cfg_;
	std::string ss_str_; // whole msg
};
