# -*- coding: utf-8 -*-
from __future__ import division, print_function
import csv, time, sys, getopt, socket, os, signal

''' Print help for this function '''
def print_help():
	msg = """ 
---------------------------------------------
               How to use:
---------------------------------------------

python file.py -i <inputfile> [-o <outputfile>] [-t <sampling time(s)>] 

The inputfile is a mandatory argument that contains the path to the file with the specified commands. 
The file has to have only entries of the syntax
time1 \t command1
time2 \t command2
etc.
where the time is given in integer or decimal seconds (e.g. 1.503 or 3) and command is a valid command taken from the text file "commands.txt" 
The times describe the absolute times (starting from the moment of execution) and therefore need to be incrementing. If two commands are given at the same time, they will in fact be executed one after the other. 

The outputfile is the path to a file where the output messages of the robot are saved to. The file is created if non-existant and if the file is existant, it is overwritten. 
If no outpufile is specified, the results are saved in output/default.txt

The sampling time is minimum time between two commands can be sent to or received from the robot. It should be chosen as big as possible while allowing for all specified times in the input file. Its default value is 0.1, allowing thus time increments of 0.1 in the input file.

-------------------------------------------- """
	print(msg)

''' Interrupt handler for stopping on KeyInterrupt '''
def signal_handler(signal, frame):
	print('Program stopped manually')
	clean_up()

''' Exit function and remove output file if empty '''
def clean_up():
	try: 
		# Check if file is empty 
		if not os.path.getsize(outputfile):
			os.remove(outputfile)
		sys.exit(1)
	except:
		sys.exit(1)

''' Open or create a file '''
def touchopen(filename,*args, **kwargs):
	fd = os.open(filename, os.O_RDWR | os.O_CREAT)
	return os.fdopen(fd, *args, **kwargs)

''' Get parameters from command line '''
def get_parameters():
	inputfile=''
	outputfile='./output/default.txt'
	t_sample = ts
	try: 
		opts,args = getopt.getopt(sys.argv[1:],"hi:o:t:",["ifile=","ofile=","t="])
	except getopt.GetoptError:
		print("error1 usage: file.py -i <inputfile> [-o <outputfile>] [-t <sampling time (s)>]")
		sys.exit(2)
	for opt, arg in opts:
		if opt in ("-h"):
			print_help()
			sys.exit(2)
		elif opt in ("-i","--ifile"):
			try:	
				with touchopen(arg,'r+'):
					inputfile = arg
			except:
				print('error3: please choose a valid input file')
				sys.exit(2)
		elif opt in ("-t","--t"):
			try:
				t_sample = float(arg)
			except:
				print('error4: Please specify a valid sampling time')
				sys.exit(2)
		elif opt in ("-o","--ofile"):
			try:
				with touchopen(arg,'r+'):
					outputfile = arg
			except:
				print('error5: please choose a valid output file')
				sys.exit(2)
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
				print('error6: non-valid time:',line['time'])
				clean_up()
			file_command = line['command']
			diff_file = file_time - file_last
			# Check if file starts with i 
			if (counter == 0) and (file_command != 'i'):
				while 1:
					choice = raw_input("warning1: file doesn't start with 'i'. Continue anyways?(yes=y,no=n) ")
					if choice == 'y':
						break
					elif choice =='n':
						clean_up()
			# Check if time increments correctly
			if diff_file < 0:
				print('error7: non-incrementing time value after ',time_array)
				clean_up()
			#�Check if time is multiple of sampling time
			elif (round(file_time/ts)%1) > 0: # use round because of rounding errors
				print('error8: not a multiple of sampling time:', file_time )
				clean_up()
			else:
			# Check if all commands are valid
				if (valid_array.count(file_command) > 0) or read_setandget(file_command): # command is valid
					time_array.append(file_time)
					command_array.append(file_command)
				else:
					print('error9: non-valid command ',file_command)
					clean_up()
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
	except (KeyboardInterrupt, SystemExit):
		print('error10: could not connect to robot, manually stopped')
		clean_up()
	except:
		print('error11: could not connect to robot')
		clean_up()

''' Send commands '''
def send_commands(time_array, command_array,outputf,s):
	counter = 0
	end_time = time_array[-1] # last element
	real_start = time.time()
	with touchopen(outputf,'wt') as f:
		c=csv.writer(f,delimiter='\t')
		for i in list(range(int(end_time/ts)+1)):
			command = command_array[counter]
			times =  time_array[counter]
			this_time = time.time()
			if (i == (int(times/ts))): # current time is one of the file times
				print('{0:15s} \t {1:5.4f} \t {2:5.4f}'.format(command, times, this_time-real_start))
				#s.send(command) # for python 2 (str)
				s.send(command.encode()) # for python 3 (bytes)
				counter += 1
				if command[0] == 'g':
					#print('write parameter and time to file')
					#c.writerow([times,command])
					data = s.recv(BUFFER_SIZE)	
					data = data.decode("utf-8") # for python 3
					c.writerow([time.time()-real_start,data])	
			time.sleep(ts-(time.time()-this_time))


''' Stop robot by default in the end '''
def default_stop():
	time.sleep(2)
	print('robot stopped automatically')
	

''' Initialization '''
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
valid_boards = ['r','l','n','o']
valid_params_motors = ['NOM_SPEED','KP','KI','KD','ACC_INC','ACC_DIV','MIN_SPEED','POS_MARG','SPEED_POS']
valid_params_neck = ['NOM_PWM','NOM_ANGLE']
valid_params_other = ['PORT','LED_GPIO','PAN_NOM_SPEED','TILT_NOM_SPEED']	
TCP_IP = '172.16.156.137'
TCP_PORT = 51717
BUFFER_SIZE = 1024
signal.signal(signal.SIGINT, signal_handler)

ts = 0.1 #default sampling time [s]
inputfile,outputfile,ts = get_parameters()
(times, commands) = read_file(inputfile,ts)
#robot_socket = setup_network()
#send_commands(times, commands, outputfile,robot_socket)
default_stop()
clean_up()
#robot_socket.close() # close socket when done communicating
