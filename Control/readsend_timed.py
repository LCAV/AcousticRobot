# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 11:43:36 2015

@author: kiki

Test reading files and timed sending 


"""
import csv
import socket
import time

TCP_IP = '172.16.156.137'
TCP_PORT = 51717
BUFFER_SIZE = 1024


#Open socket for communication
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# Send messages
#%%
last_time = 0
with open("test.txt") as f:
    c = csv.DictReader(f, delimiter='\t', fieldnames = ['time','command'], 
    skipinitialspace=True)
    for line in c:
        old_time = time.time();
        this_time = float(line['time'])
        
        time_diff = this_time - last_time
        time.sleep(time_diff)
        new_time = time.time();
        
        print("{0} \t {1:5.4f} \t {2:5.4f}".format(line['command'],time_diff,new_time-old_time))
        s.send(line['command'])
        #print(time_diff,line['time'],line['command'])
        last_time = this_time
        




# data = s.recv(BUFFER_SIZE)

# Close socket when done communicating
s.close()


