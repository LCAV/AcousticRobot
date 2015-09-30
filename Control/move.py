# -*- coding: utf-8 -*-
from __future__ import division, print_function
import csv, time, sys, getopt, socket, os, signal
import numpy as np

USAGE = '''
---------------------------------------------
               How to use:
---------------------------------------------

python file.py -i <inputfile> [-o <outputfile>] [-t <sampling time(s)>]

The inputfile is a mandatory argument that contains the path to the file with
the specified commands.
The file has to have only entries of the syntax
time1 \t command1
time2 \t command2
etc.
where the time is given in integer or decimal seconds (e.g. 1.503 or 3) and
command is a valid command taken from the text file "commands.txt"
The times describe the absolute times (starting from the moment of execution)
and therefore need to be incrementing. If two commands are given at the same time,
they will in fact be executed one after the other.

The outputfile is the path to a file where the output messages of the robot are
saved to. The file is created if non-existant and if the file is existant, it is overwritten.
If no outpufile is specified, the results are saved in output/default.txt

The sampling time is minimum time between two commands can be sent to or received
from the robot. It should be chosen as big as possible while allowing for all
specified times in the input file. Its default value is 0.1, allowing thus time
increments of 0.1 in the input file.

-------------------------------------------- '''

DEBUG = 1
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
valid_boards = ['r','l','n','o']
valid_params_motors = ['NOM_SPEED','KP','KI','KD','ACC_INC','ACC_DIV','MIN_SPEED','POS_MARG','SPEED_POS']
valid_params_neck = ['NOM_PWM','NOM_ANGLE']
valid_params_other = ['PORT','LED_GPIO','PAN_NOM_SPEED','TILT_NOM_SPEED']

def signal_handler(signal, frame):
    ''' Interrupt handler for stopping on KeyInterrupt '''
    print('Program stopped manually')
    cleanup()

def touchopen(filename,*args, **kwargs):
    ''' Open or create a file '''
    fd = os.open(filename, os.O_RDWR | os.O_CREAT)
    return os.fdopen(fd, *args, **kwargs)
def get_parameters():
    ''' Get parameters from command line '''
    inputfile=''
    outputfile='output/default.txt'
    try:
        opts,args = getopt.getopt(sys.argv[1:],"hi:o:t:",["ifile=","ofile="])
    except getopt.GetoptError:
        print("error1 usage: file.py -i <inputfile> [-o <outputfile>]")
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h"):
            print(USAGE)
            sys.exit(2)
        elif opt in ("-i","--ifile"):
            try:
                with touchopen(arg,'r+'):
                    inputfile = arg
            except:
                print('error3: please choose a valid input file')
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
    return inputfile,outputfile
def read_setandget(command):
    ''' Read p and g commands '''
    if len(command) >= 5 : # minimum length of get and set command
        if (command[1] == ' ') and (command[3] == ' '): # spaces after command and board
            if valid_boards.count(command[2]) > 0: # board is valid
                if command[0] == 'g':
                    #Â get mode
                    print('get mode')
                elif command[0] == 'p':
                    #Â set mode
                    print('set mode')
    return 1
def read_file(R,inputfile):
    ''' Read file and check if it is valid, save commands and times in dicts'''
    file_last = 0
    command_array = []
    time_array = []
    command_blocks = dict()
    time_blocks = dict()
    counter = 0
    counter_blocks = 0

    with open(inputfile) as f:
        c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
        for line in c :
            try:
                file_time = float(line['time'])
            except:
                print('error6: non-valid time:',line['time'])
                R.cleanup()
            file_command = line['command']

            diff_file = file_time - file_last
            print(diff_file,file_command)
            # Check if file starts with i
            if (counter == 0) and (file_command != 'i'):
                while 1:
                    choice = raw_input("warning1: file doesn't start with 'i'. Continue anyways?(yes=y,no=n) ")
                    if choice == 'y':
                        break
                    elif choice =='n':
                        R.cleanup()
            # Check if time increments correctly
            # add new commands and times to current array
            if (valid_array.count(file_command) > 0) or read_setandget(file_command): # command is valid
                time_array.append(file_time)
                command_array.append(file_command)
            else:
                print('error9: non-valid command ',file_command)
                R.cleanup()
            # save old command block and initialize for next one.
            if diff_file < 0:
                print('reset')
                time_blocks[counter_blocks] = time_array
                command_blocks[counter_blocks] = command_array
                time_array = []
                command_array = []
                counter_blocks += 1
                file_last = 0
            counter += 1
            file_last = file_time
        return time_blocks, command_blocks

class Robot:
    def __init__(self,IP,port,buffsize):
        self.IP=IP
        self.port=port
        self.buffsize = buffsize
        time.sleep(2)
        print('robot stopped automatically')
    def connect(self):
        ''' Setup network for Telnet communication '''
        try:
            s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s.connect((self.IP,self.port))
            self.socket = s
        except (KeyboardInterrupt, SystemExit):
            print('error10: could not connect to robot, manually stopped')
            cleanup()
        except:
            print('error11: could not connect to robot')
            cleanup()
    def cleanup(self,outputfile=''):
        ''' Close socket, delete empty outputfiles.'''
        try:
            self.socket.close()
            print('Robot socket closed succesfully')
        except:
            print('Robot not connected to start with')
        try:
            if not os.path.getsize(outputfile):
                os.remove(outputfile)
                sys.exit(1)
        except:
            sys.exit(1)
    def move(self,time_array,command_array,outputf):
        ''' Send commands from command_array at cooresponding times in time_array'''
        last_time = 0
        with touchopen(outputf,'wt') as f:
            c=csv.writer(f,delimiter='\t')
            start = time.time()
            for i,t in enumerate(time_array):
                time_diff = t - last_time
                time.sleep(time_diff)

                # send command
                command = command_array[i]
                if not DEBUG:
                    self.socket.send(command)
                c.writerow([time.time()-start,command])
                print('command written')
                print('{0:15s} \t {1:5.4f} \t {2:5.4f}'.format(command, t,time.time()-start))

                # wait for response
                if command[0]=='g':
                    if not DEBUG:
                        data = self.socket.recv(BUFFER_SIZE)
                        print('repose:',data)
                        c.writerow([time.time()-start,data])
                    print('response written')
                last_time = t
    def convert(self, pos_left, pos_right):
        ''' Convert position of encoder to mm '''
        N = 512 # Counts per revolution
        D =10 # degree of robot wheels in mm
        left_mm = pos_left/512*np.pi*D
        right_mm = pos_right/512*np.pi*D
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    # Create robot instance
    TCP_IP = '172.16.156.137'
    TCP_PORT = 51717
    BUFFER_SIZE = 1024
    R = Robot(TCP_IP,TCP_PORT,BUFFER_SIZE)

    # Get parameters
    inputfile,outputfile = get_parameters()

    # Store commands
    (times, commands) = read_file(R,inputfile)

    # Connect to robot
    if not DEBUG:
        R.connect()

    # Execute commands (in blocks)
    for i,c in enumerate(commands):
        t=times[i]
        R.move(t,c,outputfile)
        time.sleep(2)

    if not DEBUG:
        R.cleanup(outputfile)
