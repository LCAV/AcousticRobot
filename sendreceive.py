#!/usr/bin/env python

import socket

TCP_IP = '172.16.156.137'
TCP_PORT = 51717
BUFFER_SIZE = 1024
MESSAGE = "i"

# Open socket for communication
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# Send and receive messages
s.send(MESSAGE)
data = s.recv(BUFFER_SIZE)

# Close socket when done communicating
s.close()

print "received data:", data
