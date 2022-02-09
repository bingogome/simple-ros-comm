#include <ros2_side_in.hpp>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

MinimalPublisher::MinimalPublisher() : 
{

}

ROS2SideIn::ROS2SideIn(boost::asio::io_context& io_context, 
	struct ROS2SideInConfig cfg) :
	cfg_(cfg),
	socket_(io_context, udp::endpoint(udp::v4(), cfg.port_in))
{
	publisher_ = this->create_publisher<std_msgs::msg::String>(cfg_.publisher_name, 50);
	ROS2SideIn::StartReceive();
}

void ROS2SideIn::StartReceive()
{
	if (cfg_.verbose == 1)
		ROS_INFO("Waiting Transmission");
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), 
		remote_endpoint_,
		boost::bind(
			&ROSSideIn::HandleReceive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

// End port gracefully
void ROS2SideIn::EndServerClean()
{
	socket_.close();
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
		RCLCPP_INFO("Handling msg ...");
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
			RCLCPP_INFO("Ending communication node ...");
		ROS2SideIn::EndServerClean();
		throw;
	}
	else{
		// main msg handling body 
		msg_test_.data = ss_str_;
		publisher_->publish(msg_test_);
	}
}