# simple-ros-comm

## Introduction

simple-ros-comm is a package developed for the users who need a easy and flexible communication method from a hardware (or software) interface to ROS and/or ROS2.

## Dependencies
- ROS/ROS2. The package can be used in ROS or ROS2 alone, or together.
	- ROS. The package is tested on Ubuntu 20.04 with ROS noetic, but should be able to run on any typical version. Send a request to the developer if it does not work on your verson of ROS.
	- ROS2. The package is 
- Boost.asio. Boost is supported by ROS, so no external installation is needed.

## Installation
- Download the package

Customizability
You can customize your own communication protocol. Just go ahead and modify the way how the messege is handled. See ROSSideIn::HandleIncoming()

ROSSideIn
Note for ROSSideIn, I implemented it using asio and async_receive_from(), which is a blocking function. To end the node gracefully, be sure to send an ending messege to the port. Otherwise, it won't stop until a messege is received.
Multiple ROSSideIn channels are possible. I am thinking two ways: 1. multiplexing using 1 port. 2. spawn multiple nodes that each owns a different port and a ROSSideIn class.