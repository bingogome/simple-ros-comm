import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 8537

MESSAGE = b"_msg_test_123456789012345678901234567890123456789012345"
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)
sock = socket.socket(socket.AF_INET, # Internet
		socket.SOCK_DGRAM) # UDP

while True:
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
	print("Sent a message")