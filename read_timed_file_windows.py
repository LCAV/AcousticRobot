# -*- coding: utf-8 -*-
from __future__ import division, print_function
import csv, time, sys, getopt

''' Initialization '''
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
ts = 0.1 #default sampling time [s]

''' Get parameters from command line '''
def get_parameters():
	inputfile=''
	outputfile='default.txt'
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
			inputfile = arg
		elif opt in ("-t","--t"):
			try:
				t_sample = float(arg)
			except:
				print('error3: please specify a valid sampling time')
				sys.exit()
		elif opt in ("-o","--ofile"):
			try:
				with open(arg) as f:
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
	try:
		with open(inputfile) as f:
			c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
			for line in c :
				file_time = float(line['time'])
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
					print('non-incrementing time value after ',time_array)
					sys.exit()
				# Check if time is multiple of sampling time
				elif (round(file_time/ts)%1) > 0: # use round because of rounding errors
					print('not a multiple of sampling time:', file_time )
					sys.exit()
				else:
				# Check if all commands are valid
					if (valid_array.count(file_command) > 0) or read_setandget(file_command): # command is valid
						time_array.append(file_time)
						command_array.append(file_command)
					else:
						print('non-valid command ',file_command)
						sys.exit()
				counter += 1
				file_last = file_time
			return time_array, command_array
	except EnvironmentError:
		print("Please enter a valid filename")
		return 0 
		sys.exit()


''' Send commands '''
def send_commands(time_array, command_array,outputf):
	counter = 0
	end_time = time_array[-1] # last element
	real_start = time.time()
	last_time = 0 # used for timing real operations
	with open(outputf,'wb') as f:
		c=csv.writer(f,delimiter='\t')
		for i in list(range(int(end_time/ts)+1)):
			command = command_array[counter]
			times =  time_array[counter]
			this_time = time.time()
			if (i == (int(times/ts))): # current time is one of the file times
				print('{0:15s} \t {1:5.4f} \t {2:5.4f}'.format(command, times, this_time-real_start))
				counter += 1
				if command[0] == 'g':
					#print('write parameter and time to file')
					c.writerow([times,command])		
			time.sleep(ts-(time.time()-this_time))


''' Initialization '''
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
valid_boards = ['r','l','n','o']
valid_params_motors = ['NOM_SPEED','KP','KI','Kd','ACC_INC','ACC_DEC','MIN_SPEED','POS_MARG','SPEED_POS']
valid_params_neck = ['NOM_PWM','NOM_ANGLE']
valid_params_other = ['PORT','LED_GPIO','PAN_NOM_SPEED','TILT_NOM_SPEED']

ts = 0.1 #default sampling time [s]
inputfile,outputfile,ts = get_parameters()
(times, commands) = read_file(inputfile,ts)
send_commands(times, commands,outputfile)

