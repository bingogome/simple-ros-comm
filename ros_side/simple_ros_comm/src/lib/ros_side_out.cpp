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
	struct ROSSideInConfig cfg)
	: cfg_(cfg), n_(n), socket_(io_context, udp::endpoint(udp::v4(), cfg.port_out))
{
	sub_ = n_.subscribe(cfg_.subscriber_name, 100, &ROSSideOut::SubCallBack, this);
}

ROSSideOut::SubCallBack(const std_msgs::String::ConstPtr& msg)
{
	
}