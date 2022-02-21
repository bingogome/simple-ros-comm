#include <ros2_side_in.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

// Constructor
MinimalPublisher::MinimalPublisher(struct ROS2SideInConfig cfg) 
	: cfg_(cfg)
{
	node_ = rclcpp::Node::make_shared(cfg_.publisher_name);
	publisher_ = node_->create_publisher<std_msgs::msg::String>(cfg_.publisher_name, 50);
}

// The task function to be called by a std::thread that runs separately from main
void MinimalPublisher::SpinROS()
{
	rclcpp::spin(node_);
}

// The caller function of a std::thread
void MinimalPublisher::RunThread()
{
	std::thread lets_run_it(&MinimalPublisher::SpinROS, this);
	lets_run_it.detach();
}

// Publish a msg
void MinimalPublisher::PublishThis(std_msgs::msg::String msg_test)
{
	publisher_->publish(msg_test);
}

// Used by RCLCPP_INFO (ROS2 equivalent of ROS_INFO, a print function)
rclcpp::Logger MinimalPublisher::GetLogger()
{
	return node_->get_logger();
}

// Constructor
ROS2SideIn::ROS2SideIn(boost::asio::io_context& io_context, struct ROS2SideInConfig cfg) :
	cfg_(cfg),
	pub_(cfg),
	socket_(io_context, udp::endpoint(udp::v4(), cfg.port_in))
{
	pub_.RunThread();
	if (cfg_.verbose == 1)
		RCLCPP_INFO(pub_.GetLogger(), "Publisher spinning");
	ROS2SideIn::StartReceive();
}

void ROS2SideIn::StartReceive()
{
	if (cfg_.verbose == 1)
		RCLCPP_INFO(pub_.GetLogger(), "Waiting Transmission");
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), 
		remote_endpoint_,
		boost::bind(
			&ROS2SideIn::HandleReceive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

// End port gracefully
void ROS2SideIn::EndServerClean()
{
	RCLCPP_INFO(pub_.GetLogger(), "Error happened, or user interrupted");
	socket_.close();
	rclcpp::shutdown();
}

// Handle asio communication. Start new receive once finished
void ROS2SideIn::HandleReceive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/)
{
	if (!error)
	{
		ROS2SideIn::HandleIncoming();
		ROS2SideIn::StartReceive();
	}
}

// Handle msg processing.
void ROS2SideIn::HandleIncoming()
{
	if (cfg_.verbose == 1)
		RCLCPP_INFO(pub_.GetLogger(), "Handling msg ...");
	std::stringstream sscmd;
	std::stringstream ss;
	for(int i=0;i<10;i++) // header length is 10
		sscmd << recv_buffer_[i];
	for(int i=0;i<cfg_.msg_size;i++) // chars in a msg
		ss << recv_buffer_[i];
	sscmd_str_ = sscmd.str();
	ss_str_ = ss.str();

	if(sscmd_str_==cfg_.end_msg){
		if (cfg_.verbose == 1)
			RCLCPP_INFO(pub_.GetLogger(), "Ending communication node ...");
		ROS2SideIn::EndServerClean();
		throw;
	}
	else{
		// main msg handling body 
		msg_test_.data = ss_str_;
		pub_.PublishThis(msg_test_);
	}
}