# -*- coding: utf-8 -*-
##@package move
# move
# ==============
# Contains Robot class
#
#created by Frederike Duembgen, July 2015
from __future__ import division, print_function
import csv, time, sys, getopt, socket, os, signal
import numpy as np
USAGE = '''
---------------------------------------------
               How to use:
---------------------------------------------

python file.py -i <inputfile> [-d] manual drive with input
python file.py -a [-d] manual drive without input

The file has to have only entries of the syntax
time1 \t command1
time2 \t command2
etc.


the output will be saved in the "output/" folder with the same name as the
inputfile but with appendix _odo for odometry data and _tim for timing data

Add -d to run program in "simulation mode" (without connecting to robot)
-------------------------------------------- '''

DEBUG = 0
N = 512 # Counts per revolution
R_wheels =10 # radius of robot wheels in mm
D=30 # distance between two wheels in mm
valid_array = ['i','f','b','l','r','e','v','u','d','a','o','c','sh','sa','s','pl','pr','g','tu','td','z'] # valid commands
valid_boards = ['r','l','n','o']
valid_params_motors = ['NOM_SPEED','KP','KI','KD','ACC_INC','ACC_DIV','MIN_SPEED','POS_MARG','SPEED_POS']
valid_params_neck = ['NOM_PWM','NOM_ANGLE']
valid_params_other = ['PORT','LED_GPIO','PAN_NOM_SPEED','TILT_NOM_SPEED']

TCP_IP = '172.16.156.137'
TCP_PORT = 51717
BUFFER_SIZE = 1024

def signal_handler(signal, frame):
    ''' Interrupt handler for stopping on KeyInterrupt '''
    print('Program stopped manually')
    sys.exit(2)

def touchopen(filename,*args, **kwargs):
    ''' Open or create a file '''
    fd = os.open(filename, os.O_RDWR | os.O_CREAT)
    return os.fdopen(fd, *args, **kwargs)

def mode_classification():
    global DEBUG
    try:
        opts,args = getopt.getopt(sys.argv[1:],"hi:da",["ifile="])
    except getopt.GetoptError:
        print("error1", USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h"):
            print(USAGE)
            sys.exit(2)
        elif opt in ("-d"):
            DEBUG=1
        elif opt in ("-i","--ifile"):
            try:
                with touchopen(arg,'r+'):
                    inputfile = arg
            except:
                print('error: please choose a valid input file')
                sys.exit(2)
            mode='withinput'
            return mode
        elif opt in ('-a'):
            mode='withoutinput'
            return mode

def get_parameters_with(mode):
    ''' Get parameters from command line '''
    global DEBUG
    inputfile=''
    output_result='output/move.txt'
    
    try:
        opts,args = getopt.getopt(sys.argv[1:],"hi:d",["ifile="])
    except getopt.GetoptError:
        print("error1", USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h"):
            print(USAGE)
            sys.exit(2)
        elif opt in ("-d"):
            DEBUG=1
        elif opt in ("-i","--ifile"):
            try:
                with touchopen(arg,'r+'):
                    inputfile = arg
            except:
                print('error: please choose a valid input file')
                sys.exit(2)
    if inputfile == '':
        print('error: please choose a valid input file')
        sys.exit(2)
    print('Input file is ',inputfile)
    output_result='output/move.txt'
    return inputfile, output_result

def get_parameters_without(mode):
    ''' Get parameters from command line '''
    global DEBUG
    inputfile=''
    output_result='output/move.txt'   
    try:
        opts,args = getopt.getopt(sys.argv[1:],"hda")
    except getopt.GetoptError:
        print("error1", USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h"):
            print(USAGE)
            sys.exit(2)
        elif opt in ("-d"):
            DEBUG=1
        elif opt in ('-a'):
            mode='withoutinput'
    print('Input file is ',inputfile)
    output_result='output/move.txt'
    return output_result

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
    command_array = []
    time_array = []
    command_blocks = dict()
    time_blocks = dict()
    counter_blocks = -1
    last_time = 0
    with open(inputfile) as f:
        c=csv.DictReader(f,delimiter = '\t', fieldnames = ['time','command'],skipinitialspace=True)
        for line in c :
            print(line)
            # read new time and command
            try:
                file_time = float(line['time'])
            except:
                print('error6: non-valid time:',line['time'])
                R.cleanup()
            file_command = line['command']

            if file_time != 0 and file_time-last_time<0:
                print('error7: non-incrementing time values:',file_time,last_time)
                R.cleanup()
            # save old command block and initialize for next one.
            elif file_time == 0:
                if counter_blocks != -1:
                    time_blocks[counter_blocks] = time_array
                    command_blocks[counter_blocks] = command_array
                    time_array = []
                    command_array = []
                counter_blocks += 1

            # Add new valid commands and times to current array
            if (valid_array.count(file_command) > 0) or read_setandget(file_command):
                time_array.append(file_time)
                command_array.append(file_command)
                last_time = file_time
            else:
                print('error9: non-valid command ',file_command)
                R.cleanup()
        # Add last array to block
        time_blocks[counter_blocks] = time_array
        command_blocks[counter_blocks] = command_array
        choice = input("proceed with above listed commands?(yes=y,no=n)")
        if (choice=='y'):
            return time_blocks, command_blocks
        else:
            R.cleanup()
class Robot:
    '''
    This class implements functions used to control robot and read its encoder data.
    '''
    def __init__(self,IP=TCP_IP,port=TCP_PORT,buffsize=BUFFER_SIZE):
        self.IP=IP
        self.port=port
        self.buffsize = buffsize
    def connect(self):
        ''' Setup network for Telnet communication '''
        try:
            s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s.connect((self.IP,self.port))
            self.socket = s
        except (KeyboardInterrupt, SystemExit):
            print('error10: could not connect to robot, manually stopped')
            self.cleanup()
        except:
            print('error11: could not connect to robot')
            self.cleanup()
    def cleanup(self,output_tim='',output_odo=''):
        ''' Close socket, delete empty outputfiles'''
        try:
            self.socket.close()
            print('Robot socket closed succesfully')
        except:
            print('Did not connect to robot')
        try:
            if not os.path.getsize(output_tim):
                os.remove(output_tim)
                sys.exit(1)
            if not os.path.getsize(output_odo):
                os.remove(output_odo)
                sys.exit(1)
        except:
            sys.exit(1)
    def move(self,time_array,command_array,outputf):

        position = dict()
        motors=["l","r"]
        position[motors[0]]=-1
        position[motors[1]]=-1

        ''' Send commands from command_array at cooresponding times in time_array'''
        last_time = 0
        with touchopen(outputf,'a') as f:
            c=csv.writer(f,delimiter='\t')
            start = time.time()
            for i,t in enumerate(time_array):
                time_diff = t - last_time
                time.sleep(time_diff)

                # send command
                command = command_array[i]
                if not DEBUG:

                    self.socket.send(command.encode())
                c.writerow([command])
                print('{0:15s} \t {1:5.4f} \t {2:5.4f}'.format(command, t,time.time()-start))

                # wait for response
                if command[0]=='g':
                    if not DEBUG:
                        data = self.socket.recv(BUFFER_SIZE)
                        print('data',data)
                        c.writerow([round(time.time()-start,4),data])
                last_time = t

                for motor in motors:
                    cmd = "g "+motor+" ACT_POS"

                    if not DEBUG:
                        self.socket.send(cmd.encode())
                #wait for response
                    found = 0
                    if not DEBUG:
                        while not found:
                            data = self.socket.recv(BUFFER_SIZE)
                            if data.find(cmd.encode())!= -1:
                                pos = int(data.replace(cmd.encode(),b""))
                                found = 1
                        #pos = data
                        #found = 1
                        position[motor]=pos
                c.writerow([position[motors[0]],position[motors[1]]])

    def driven(self,outputf):
        
        command = 'i'
        position = dict()
        motors=["l","r"]
        position[motors[0]]=-1
        position[motors[1]]=-1
        t0 = time.time()
        
        command0 = input('Drive mode begins, please enter your command:')
        while command0 != 'end':
            if (command0 != 'f' and command0 != 'b' and command0 != 'l' and command0 != 'r' and command0 != 'end' and command0 != 's'):
                print('wrong command and try another one')
                
            else:
                command = command0
                while command != 'end':
                    with touchopen(outputf,'a') as f:
                        c=csv.writer(f,delimiter='\t')                    
                        # send command
                        if not DEBUG:
                            self.socket.send(command.encode())
                        c.writerow([command])
                        t1 = time.time()
                        print('{0:15s} \t {1:5.4f} '.format(command, t1-t0))

                        # wait for response
                        if command[0]=='g':
                            if not DEBUG:
                                data = self.socket.recv(BUFFER_SIZE)
                                print('data',data)
                                c.writerow([round(time.time()-start,4),data])

                        for motor in motors:
                            cmd = "g "+motor+" ACT_POS"

                            if not DEBUG:
                                self.socket.send(cmd.encode())
                        #wait for response
                            found = 0
                            if not DEBUG:
                                while not found:
                                    data = self.socket.recv(BUFFER_SIZE)
                                    if data.find(cmd.encode())!= -1:
                                        pos = int(data.replace(cmd.encode(),b""))
                                        found = 1
                                position[motor]=pos
                        c.writerow([position[motors[0]],position[motors[1]]])
                        break
                    break
                        
            command0=input('type another command as you want:...')

    def activate(self):
        ''' Set motors back into control mode (to be called after every stop) '''
        if not DEBUG:
            self.socket.send("p l CONTROL 2")
            time.sleep(1)
            self.socket.send("p r CONTROL 2")
        return 1



if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    R = Robot(TCP_IP,TCP_PORT,BUFFER_SIZE)
    mode = mode_classification()
    
    if mode == 'withinput':
        inputfile,output_result= get_parameters_with(mode)
        if (DEBUG):
            print("running in debug mode")
        else:
            print("running in real mode")
        (times, commands) = read_file(R,inputfile)
        if not DEBUG:
            R.connect()
        for i,c in commands.items():
            print("starting new movement")
            t=times[i]
            R.move(t,c,output_result)
            print("movement done")
            time.sleep(2)
            if not DEBUG:
                R.cleanup()
    
    if mode == 'withoutinput':
        output_result = get_parameters_without(mode)
        if (DEBUG):
            print("running in debug mode")
        else:
            print("running in real mode")
        if not DEBUG:
            R.connect()
        print("starting new movement")
        R.driven(output_result)
        print("movement done")
        time.sleep(2)
        if not DEBUG:
            R.cleanup()
