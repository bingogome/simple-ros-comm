# simple-ros-comm
![alt text](https://github.com/bingogome/documents/blob/main/simple-ros-comm/design.drawio.png)
## Introduction

**simple-ros-comm** is a package developed for the users who need a easy and flexible communication method from a hardware (or software) interface to ROS and/or ROS2. The package is developed using C++ on the ROS/ROS2 side, and is language invariant on the machine side (the side that you want ROS/ROS2 talks to). 
It allows the user to create customized "data protocol" by editing how the transmitted string is encoded. It does not support data type except for string at the moment.

## Dependencies
- ROS/ROS2. The package can be used in ROS or ROS2 alone, or together.
	- ROS. The package is tested on Ubuntu 20.04 with ROS noetic, but should be able to run on any typical version. Send a request to the developer if it does not work on your verson of ROS.
	- ROS2. The package is tested on Ubuntu 20.04 with ROS galactic, but should be able to run on any typical version. Send a request to the developer if it does not work on your verson of ROS.
- Boost.asio. Boost is supported by ROS and ROS2, so no external installation is needed.

## Instruction

### ROS1 - Send some data into ROS1
- Clone the repository to a directory. The cloned directory is *DIR_UPPER/simple-ros-comm*.
- Create a symbolic link (similar to shortcut as in Windows) to your ROS workspace *DIR_WS/src* folder. (Or simply copy it to there if you don't need to edit the code).
```
cd DIR_WS/src
ln -s DIR_UPPER/simple-ros-comm/ros_side/simple_ros_comm DIR_WS/src/simple_ros_comm
```
- Build the package
```
cd DIR_WS
source /opt/ros/noetic/setup.bash
catkin build
```
- Start talking
```
roscore
# on new terminal
cd DIR_WS
source /opt/ros/noetic/setup.bash
source DIR_WS/devel/setup.bash
rosrun simple_ros_comm node_ros_side_in # the node to send encoded command to ROS

```
- **Remember to close the port by sending the port an ending message. See *config_ros.yaml.***
- Change the port, ending message, and more in configuration file *config_ros.yaml* as needed.

### ROS2 - Send some data into ROS2
- Clone the repository to a directory. The cloned directory is *DIR_UPPER/simple-ros-comm*.
- Create a symbolic link (similar to shortcut as in Windows) to your ROS workspace *DIR_WS/src* folder. (Or simply copy it to there if you don't need to edit the code).
```
cd DIR_WS/src
ln -s DIR_UPPER/simple-ros-comm/ros_side/simple_ros2_comm DIR_WS/src/simple_ros2_comm
```
- Build the package
```
cd DIR_WS
source /opt/ros/galactic/setup.bash
colcon build
```
- Start talking
```
cd DIR_WS
source /opt/ros/galactic/setup.bash
source DIR_WS/install/setup.bash
ros2 run simple_ros2_comm node_ros2_side_in # the node to send encoded command to ROS2

```
- **Remember to close the port by sending the port an ending message. See *config_ros2.yaml.***
- Change the port, ending message, and more in configuration file *config_ros2.yaml* as needed.

### A demo showing ROSIn and ROS2In working together
https://github.com/bingogome/documents/blob/main/simple-ros-comm/ros_n_ros2in-matlab-demo.mkv

## Customizability
You can customize your own communication protocol. Just go ahead and modify the way how the messege is handled. See ROSSideIn::HandleIncoming()

## ROSSideIn
Note for ROSSideIn, I implemented it using asio and async_receive_from(), which is a blocking function. To end the node gracefully, be sure to send an ending messege to the port. Otherwise, it won't stop until a messege is received.
Multiple ROSSideIn channels are possible. I am thinking two ways: 1. multiplexing using 1 port. 2. spawn multiple nodes that each owns a different port and a ROSSideIn class.