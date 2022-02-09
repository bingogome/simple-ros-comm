#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class ROSSideIn
{

public: 
	// constructor and start receiving
	ROSSideIn(ros::NodeHandle& n, boost::asio::io_context& io_context);

private:

	void StartReceive();
	void EndServerClean();
	void HandleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void HandleIncoming();

	// ros related members
	ros::NodeHandle& n_;
	ros::Publisher pubTest_; // can be action, service. Use topic as a demo

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 55> recv_buffer_; // TODO: make this configurable 55 chars in an encoded command
	// TODO: make socket array so that multiple channels are possible (or dynamic expansion)
	// TODO: make TCP optional

}