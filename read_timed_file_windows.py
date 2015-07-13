from __future__ import division, print_function
import csv, time, sys, getopt


#Get required file from command line
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


#%%
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


