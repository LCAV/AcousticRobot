# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""
Created on Thu Jul  9 11:43:36 2015

@author: kiki

Test reading files and timed sending 
with command line arguments

"""
import csv, time, sys, getopt
from __future__ import division, print_function


#Get required file from command line
inputfile = ''
try:
	opts,args = getopt.getopt(sys.argv[1:],"i:",["ifile="])
except getopt.GetoptError:
	print("error1: type read_timed_file.py -i <inputfile>")
	sys.exit(2)
for opt, arg in opts:
	if opt == '-h':
		print("error2: type read_timed_file.py -i <inputfile>")
		sys.exit(1)
	elif opt in ("-i","--ifile"):
		inputfile = arg
print('Input file is', inputfile)

#%%
try:
	last_time = 0
	with open(inputfile) as f:
    		c=csv.DictReader(f, delimiter='\t', fieldnames = ['time','command'],skipinitialspace=True)
    		for line in c:
        		old_time = time.time()
        		this_time = float(line['time'])
        
        		time_diff = this_time - last_time
        		time.sleep(time_diff)
        		new_time = time.time()
        
       		 	#print('{0} \t {1:5.4f} \t {2:5.4f}'.format(line['command'],time_diff,new_time-old_time))
			last_time = this_time
except EnvironmentError:
	print("Please enter a valid filename")





