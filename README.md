# simple-ros-comm

Customizability
You can customize your own communication protocol. Just go ahead and modify the way how the messege is handled. See ROSSideIn::HandleIncoming()

ROSSideIn
Note for ROSSideIn, I implemented it using asio and async_receive_from(), which is a blocking function. To end the node gracefully, be sure to send an ending messege to the port. Otherwise, it won't stop until a messege is received.
Multiple ROSSideIn channels are possible. I am thinking two ways: 1. multiplexing using 1 port. 2. spawn multiple nodes that each owns a different port and a ROSSideIn class.