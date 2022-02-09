#include <ros_side_in.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

ROSSideIn::ROSSideIn(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	int port_in) :
	n_(n), 
	socket_(io_context, udp::endpoint(udp::v4(), port_in))
{
	ROSSideIn::StartReceive();
}

void ROSSideIn::StartReceive()
{
	ROS_INFO("Waiting Transmission");
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), 
		remote_endpoint_,
		boost::bind(
			&KSTUDPServer::handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void ROSSideIn::EndServerClean()
{
	socket_.close();
}

void ROSSideIn::HandleReceive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/)
{
	if (!error)
	{
		ROSSideIn::HandleIncoming();
		ros::spinOnce();
		ROSSideIn::StartReceive();
	}
}

void ROSSideIn::HandleIncoming()
{

}