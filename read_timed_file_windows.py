from __future__ import division, print_function
import csv, time, sys, getopt


#Get required file from command line
inputfile=''
try: 
	opts,args = getopt.getopt(sys.argv[1:],"i:",["ifile="])
except getopt.GetoptError:
	print("error1: type read_timed_file.py -i <inputfile>")
	sys.exit(2)




