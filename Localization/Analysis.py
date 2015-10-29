# -*- coding: utf-8 -*-
'''
Analysis.py module
==============

Contains functions to analyze room impulse response
'''
import sys
import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np

# Odometry
N = 512 # Counts per revolution
G = 43/1 # Gear ration
N_tot = N*G
R_wheels =150. # radius of robot wheels in mm
D=710. # distance between two wheels in mm

# RIR
Fs = 44100. # sampling rate
T = 20 # seconds of interest
M = 300000 # width Hann window
zoom1=1. # seconds zoom1
zoom2=0.1 #seconds zoom2

def get_parameters():
    ''' Get parameters from command line '''
    if len(sys.argv) < 3:
        print("Plays a wave file.\n\nUsage: %s u_file.wav y_file.wav [y_file2.wav y_file3.wav ...] out_dir" % sys.argv[0])
        sys.exit(-1)
    u_file = sys.argv[1]
    y_files = sys.argv[2:-1]
    out_dir = sys.argv[-1]
    return u_file,y_files,out_dir
def get_RIR(u_file,y_file,out_dir):
    ''' calculates the RIR from in-and output .wav files and saves plots

    _Parameters_:
    u_file: path to sine sweep
    y_file: path to recorded response
    out_dir: folder where results should be saved

    _Returns_:
    (t,h_filtered): time vector and impulse response
    (f,H_filtered): frequency vector and FFT of impulse response
    '''
    # extract number
    number = int(y_file.partition('/')[-1][0])

    # Read input files
    name='RIR_'+str(number)
    Fs,u = wavfile.read(u_file)
    print(Fs)
    Fs,y = wavfile.read(y_file)
    Fs=float(Fs)
    # Deconvolute signals
    t = np.linspace(0,T,T*Fs) # time vector
    N=2*max(len(u),len(y))
    u_long = np.zeros(N)
    y_long = np.zeros(N)
    u_long[:len(u)]=u
    y_long[:len(y)]=y
    Y = abs(np.fft.rfft(y_long,n=N))
    U = abs(np.fft.rfft(u_long,n=N))
    H = Y/U

    # Apply window
    hann=np.zeros(H.shape)
    hann[:M] = np.hanning(2*M)[M:]
    H_filtered=H*hann

    # Create impulse response
    h_filtered = np.fft.irfft(H_filtered)
    f = np.fft.rfftfreq(N, d=1/Fs)

    # Show and save results
    plt.close('all')
    plt.figure(1),plt.plot(t,h_filtered[:len(t)]*1000),plt.axis('tight')
    plt.xlabel('t [s]'),plt.ylabel('h'),plt.title('RIR_'+str(number))
    plt.savefig(out_dir+'RIR_'+str(number))
    plt.figure(2),plt.plot(t,h_filtered[:len(t)]*1000),plt.axis([0,zoom1,-0.05,0.05])
    plt.xlabel('t [s]'),plt.ylabel('h'),plt.title('RIR_'+str(number)+'_zoom1')
    plt.savefig(out_dir+'RIR_'+str(number)+'_zoom1')
    plt.figure(3),plt.plot(t,h_filtered[:len(t)]*1000),plt.axis([0,zoom2,-0.1,0.1])
    plt.xlabel('t [s]'),plt.ylabel('h'),plt.title('RIR_'+str(number)+'_zoom2')
    plt.savefig(out_dir+'RIR_'+str(number)+'_zoom2')
    plt.figure(4),plt.plot(f,H),plt.plot(f,hann*max(H))
    plt.xlabel('f (Hz)'),plt.ylabel('|H(f)|'),plt.title('Unfiltered H and applied Filter')
    plt.savefig(out_dir+'H_'+str(number))

    return np.array([t,h_filtered]),np.array([f,H_filtered])
def save_RIR(y_file,out_dir,h):
    ''' Saves RIR in binary file

    _Parameters_:
    y_file: input file for rir response (used for naming only)
    out_dir: output directory for impulse response
    h: vector of impulse response

    _Returns_: Nothing
    '''
    wav_file = y_file.replace('audio','rir')
    wav_file = wav_file.split('/')[-1]
    wav_file = out_dir+wav_file
    wavfile.write(wav_file,Fs,h)

def odometry(self, enc_left, enc_right,x1,y1,theta1):
    '''
    Calculates new position x2,y2, based on encoder measures and
    last position x1,y1

    _Parameters_:
    enc_left, enc_right: encoder positions
    x1,y1,theta1: last position

    _Returns_:
    x2,y2,theta2: new position
    '''
    l_left = enc_left/N*np.pi*R_wheels
    l_right = enc_right/N*np.pi*R_wheels

    theta2 = (l_left-l_right)/D+theta1
    r=abs(D/2*(l_left+l_right)/(l_left-l_right))

    # calculate new positions
    x2 = x1 + r*(cos(theta2-theta1)-1)
    y2 = y2 + r*(sin(theta2-theta1)-1)
    return x2,y2,theta2

if __name__ == "__main__":
    #Room Impulse Response
    u_file,y_files,out_dir=get_parameters()
    for y_file in y_files:
        T,H=get_RIR(u_file,y_file,out_dir)
        save_RIR(y_file,out_dir,T[1])
    # Odometry
    x1 = 2118
    y1 = 1975
    theta1 = np.pi
    real = np.array([[2118,1975],[1829,1948],[1544,1935]])
    encoders = np.array([[0.,0.],[-6825.,6699.],[-12957.,13392.]])

    positions = []
    rs=[]
    dthetas=[]
    positions.append([round(real[0][0]),round(real[0][1]),round(theta1,4)])
    enc_old=encoders[0]
    for i in range(len(encoders)-1):
        enc = encoders[i+1]-encoders[i]

        x1 = positions[i][0]
        y1 = positions[i][1]
        theta1 = positions[i][2]
        # conversion in mm
        l_left = enc[0]/N_tot*2*np.pi*R_wheels
        l_right = -enc[1]/N_tot*2*np.pi*R_wheels

        dtheta=(l_left-l_right)/D
        theta2 = dtheta+theta1
        r=D/2*(l_left+l_right)/(l_left-l_right)

        # calculate new positions
        # TODO: Check the sign of these for cos(theta2-thea1 negative)
        y2 = y1 + abs(r)*(np.cos(theta2-theta1)-1)
        x2 = x1 + r*(np.sin(theta2-theta1))
        positions.append([round(x2),round(y2),round(theta2,4)])
        dthetas.append(dtheta)
        rs.append(r)
    pos = np.array(positions)
    plt.figure(5)
    plt.plot(pos[:,0],pos[:,1],label='odometry',marker='*'),
    plt.plot(real[:,0],real[:,1],label='real',marker='*')
    plt.xlabel('x [mm]'),plt.ylabel('y [mm]'),plt.legend()
    plt.title('Odometry precision study')
    plt.axis('equal'),plt.show(block=False)
    plt.savefig(out_dir+'odometry')


