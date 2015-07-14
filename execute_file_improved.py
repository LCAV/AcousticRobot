# -*- coding: utf-8 -*-
from __future__ import division, print_function
import csv, time, sys, getopt, socket


''' Get parameters from command line '''
def get_parameters():
	inputfile=''
	outputfile='./output/default.txt'
	t_sample = ts
	try: 
		opts,args = getopt.getopt(sys.argv[1:],"i:o:t:",["ifile=","ofile=","t="])
	except getopt.GetoptError:
		print("error1: syntax is file.py -i <inputfile> -o <outputfile> -t <sampling time [s]>")
		sys.exit()
	for opt, arg in opts:
		if opt == '-h':
			print("error2: syntax is file.py -i <inputfile> -o <outputfile> -t <sampling time [s]>")
			sys.exit()
		elif opt in ("-i","--ifile"):
			try:
				with open(arg,'r+'):
					inputfile = arg
			except:
				print('error3: please choose a valid input file')
				sys.exit()
		elif opt in ("-t","--t"):
			try:
				t_sample = float(arg)
			except:
				print('error3: please specify a valid sampling time')
				sys.exit()
		elif opt in ("-o","--ofile"):
			try:
				with open(arg,'r+'):
					outputfile = arg
			except:
				print('error4: please choose a valid output file')
				sys.exit()
	print('Input file is',inputfile)
	print('Output file is',outputfile)
	print('Sampling time is',t_sample)
	return inputfile,outputfile,t_sample
	
''' Read p and g commands '''
def read_setandget(command):
	if len(command) >= 5 : # minimum length of get and set command
		if (command[1] == ' ') and (command[3] == ' '): # spaces after command and board
			if valid_boards.count(command[2]) > 0: # board is valid
				if command[0] == 'g':
					# get mode
					print('get mode')
				elif command[0] == 'p':
					# set mode
					print('set mode')
	return 1			

''' Read file and check if it is valid '''
def read_file(inputfile,ts):
	file_last = 0
	command_array = []
	time_array = []
	counter = 0
	with open(inputfile) as f:
		c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
		for line in c :
			try:
				file_time = float(line['time'])
			except:
				print('error4: non-valid time:',line['time'])
				sys.exit()
			file_command = line['command']
			diff_file = file_time - file_last
			# Check if file starts with i 
			if (counter == 0) and (file_command != 'i'):
				while 1:
					choice = raw_input("warning1: file doesn't start with 'i'. Continue anyways?(yes=y,no=n) ")
					if choice == 'y':
						break
					elif choice =='n':
						sys.exit()
			# Check if time increments correctly
			if diff_file < 0:
				print('error5: non-incrementing time value after ',time_array)
				sys.exit()
			# Check if time is multiple of sampling time
			elif (round(file_time/ts)%1) > 0: # use round because of rounding errors
				print('error6: not a multiple of sampling time:', file_time )
				sys.exit()
			else:
			# Check if all commands are valid
				if (valid_array.count(file_command) > 0) or read_setandget(file_command): # command is valid
					time_array.append(file_time)
					command_array.append(file_command)
				else:
					print('error7: non-valid command ',file_command)
					sys.exit()
			counter += 1
			file_last = file_time
		return time_array, command_array

''' Setup network for Telnet communication '''
def setup_network():
	# Open socket for communication
	try:
		s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		s.connect((TCP_IP,TCP_PORT))
		return s
	except:
		print('error8: could not connect to robot')
	

''' Send commands '''
def send_commands(time_array, command_array,outputf,s):
	counter = 0
	end_time = time_array[-1] # last element
	real_start = time.time()
	with open(outputf,'wt') as f:
		c=csv.writer(f,delimiter='\t')
		for i in list(range(int(end_time/ts)+1)):
			command = command_array[counter]
			times =  time_array[counter]
			this_time = time.time()
			if (i == (int(times/ts))): # current time is one of the file times
				print('{0:15s} \t {1:5.4f} \t {2:5.4f}'.format(command, times, this_time-real_start))
				s.send(command)
				counter += 1
				if command[0] == 'g':
					#print('write parameter and time to file')
					#c.writerow([times,command])
					data = s.recv(BUFFER_SIZE)	
					c.writerow([time.time()-real_start,data])	
			time.sleep(ts-(time.time()-this_time))


''' Stop robot by default '''
def default_stop():
	time.sleep(2)
	print('robot stopped automatically')
	

''' Initialization '''
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
valid_boards = ['r','l','n','o']
valid_params_motors = ['NOM_SPEED','KP','KI','Kd','ACC_INC','ACC_DEC','MIN_SPEED','POS_MARG','SPEED_POS']
valid_params_neck = ['NOM_PWM','NOM_ANGLE']
valid_params_other = ['PORT','LED_GPIO','PAN_NOM_SPEED','TILT_NOM_SPEED']	
TCP_IP = '172.16.156.137'
TCP_PORT = 51717
BUFFER_SIZE = 1024

ts = 0.1 #default sampling time [s]
inputfile,outputfile,ts = get_parameters()
(times, commands) = read_file(inputfile,ts)
robot_socket = setup_network()
send_commands(times, commands, outputfile,robot_socket)
default_stop()
robot_socket.close() # close socket when done communicating
