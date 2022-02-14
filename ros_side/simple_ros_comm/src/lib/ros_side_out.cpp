#include <ros_side_out.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

ROSSideOut::ROSSideOut(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	struct ROSSideOutConfig cfg)
	: cfg_(cfg), n_(n), socket_(io_context), remote_endpoint_(udp::v4(), cfg.port_out)
	// : cfg_(cfg), n_(n), socket_(io_context, udp::endpoint(udp::v4(), cfg.port_out))
{
	socket_.open(udp::v4());
	sub_ = n_.subscribe(cfg_.subscriber_name, 100, &ROSSideOut::SubCallBack, this);
}

void ROSSideOut::SubCallBack(const std_msgs::String::ConstPtr& msg)
{
	// boost::system::error_code ignored_error;
	ss_str_ = msg->data.c_str();
	// socket_.send_to(boost::asio::buffer(ss_str_),
	// 	remote_endpoint_, 0, ignored_error);
	socket_.send_to(boost::asio::buffer(ss_str_),remote_endpoint_);
	ROS_INFO_STREAM("Msg: "<<ss_str_);
}