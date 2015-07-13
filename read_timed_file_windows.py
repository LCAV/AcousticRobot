from __future__ import division, print_function
import csv, time, sys, getopt

#%% Initialization
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands

#%% Get input file from command line
inputfile=''
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
print('Input file is',inputfile)


#%% Check if file is valid
try:
	file_last = 0
	command_array = []
	time_array = []
	counter = 0
	with open(inputfile) as f:
		c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
		for line in c :
			file_time = float(line['time'])
			file_command = line['command']
			diff_file = file_time - file_last
			
			# Check if starts with i
			if counter == 0 && file_command != 'i'
				
				choice = raw_input("warning1: file doesn't start with 'i'. Still continue?(yes=1,no=0) ")
				
				

			
			# Check increments
			if diff_file < 0:
				print('non-incrementing time value after ',time_array)
			else:
			# Check if valid command
				if valid_array.count(file_command) > 0:
					time_array.append(file_time)
					command_array.append(file_command)
				else:
					print('non-valid command ',file_command)
			counter += 1

except EnvironmentError:
	print("Please enter a valid filename")




#%% Send out and receive commands
try:
	file_last = 0
	with open(inputfile) as f:
		c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
		for line in c : 
			real_last = time.time()
			file_this = float(line['time'])
			diff_file = file_this - file_last
			time.sleep(diff_file)
			real_this = time.time()

			print('{0} \t {1:5.4f} \t {2:5.4f}'.format(line['command'],diff_file,real_this-real_last))
			file_last = file_this
except EnvironmentError:
	print("Please enter a valid filename")


