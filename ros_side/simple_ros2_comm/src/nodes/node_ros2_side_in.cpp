#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  boost::asio::io_context io_context;
  rclcpp::spin(std::make_shared<ROS2SideIn>());
  rclcpp::shutdown();
  return 0;
}