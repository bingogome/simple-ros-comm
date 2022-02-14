#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_test_ros_side_out");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/SimpleRosComm/msg_to_send", 1000);
	ros::Rate loop_rate(0.5);

	while (ros::ok())
	{
		std::string s;
		s = "_msg_test_12345678901234567890123456789012345678901234\n";
		std_msgs::String msg;
		msg.data = s;
		ROS_INFO("Sent");
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
	

}