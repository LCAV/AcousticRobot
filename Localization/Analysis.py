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

from scipy.signal import firwin, kaiserord, filtfilt

# Odometry
N = 512 # Counts per revolution
G = 43/1 # Gear ratio
N_tot = N*G
R_wheels =150. # radius of robot wheels in mm
D=710. # distance between two wheels in mm

# RIR
MIN_F = 100 # minimum of sine sweep
MAX_F = 20000  # maximum of sine sweep
Fs = 44100. # sampling rate
T = 10 # seconds of interest for room impulse response
WIDTH_ROOM = 6500 #in mm
HEIGHT_ROOM = 6000 #in mm
N_WALLS = 4
C = 343200 # speed of sound at 20C, dry air, in mm/s
# CROSS CORRELATION
MAX_LENGTH = Fs*2
TLAT = 0.1454 # Latency time, found with 660mm distance test

class Analysis():
    def __init__(self,out_dir,input_wav='', output_wav_list='', output_enc='',
                 output_real='',output_vis=''):
        '''
        _Parameters_:
        out_dir: folder where results should be saved
        input_wav: audio input file with path (most likely sine sweep)
        output_wav_list: list of audio output files
        output_end: encoder output file
        output_real: real positions output file
        output_vis: visual localization output file

        '''
        if out_dir[-1]!='/':
            out_dir = out_dir+'/'
        self.out_dir = out_dir
        self.input_wav = input_wav
        self.output_wav_list = output_wav_list
        self.output_enc = output_enc
        self.output_real = output_real
        self.output_vis = output_vis
        self.U=''
    def get_TOA(self):
        '''
        Calculate matrix of estimated time of arrival based on distances
        to wall, calculated from real positions and room width and height.

        _Returns_:
            np.Array U of size K*N_steps, where N_steps is number of steps and
            K is number of walls. Therefore, the rows correspond the response
            times from the K walls and the columns correspond to steps.
        '''
        output_real = np.loadtxt(self.output_real,dtype=np.float32)
        U = np.zeros((N_WALLS,output_real.shape[0]))
        for i in range(0,output_real.shape[0]):
            U[0,i]=(HEIGHT_ROOM-output_real[i,1])*2/C
            U[1,i]=(WIDTH_ROOM-output_real[i,0])*2/C
            U[2,i]=output_real[i,1]*2/C
            U[3,i]=output_real[i,0]*2/C
        self.U = U+TLAT
        return U
    def get_RIR(self):
        ''' calculates the RIR from self.input_wav and all corresponding
        self.output_wav_list files and saves plots in self.out_dir with the
        same step_number and channel number in name.
        (for example: "out_dir/0_1_RIR_....png" saves impulse response at
        step 0 of channel 1.

        _Returns_:
        (t,h_filtered): time vector and impulse response
        (f,H_filtered): frequency vector and FFT of impulse response
        '''
        Fs,u = wavfile.read(self.input_wav)
        for output_wav in self.output_wav_list:
            step_number = int(output_wav.partition('/')[-1][0])
            channel_number = int(output_wav[-5])
            # Read input files
            name=self.out_dir+str(step_number)+'_'+str(channel_number)+'_RIR_'
            Fs2,y = wavfile.read(output_wav)
            if Fs2 != Fs:
                print("Warning: Sampling rate from input doens't match output:",
                      Fs,Fs2)
            # reshape to one-channel array (can be removed once Audio.py
            # is fixed
            y = y.reshape((-1))
            Fs=float(Fs)
            # Deconvolute signals
            t = np.linspace(0,T,T*Fs) # time vector
            N=2*max(len(u),len(y))
            f = np.fft.rfftfreq(N, d=1/Fs)
            Y = np.fft.rfft(y,n=N) # automatically adds 0s until N
            U = np.fft.rfft(u,n=N)

            H = np.zeros((max(len(u),len(y))+1),dtype=np.complex)
            print(U.shape[0],Y.shape[0],H.shape[0],N)
            # find the length where U is not 0. (or bigger than 1000)
            lengthU = U.shape[0]
            thresh = 1000
            for i in range(lengthU-1,-1,-1):
                if abs(U[i])>thresh:
                    print("non-zero length U", lengthU)
                    lengthU = i
                    break
            # alternative: calculate length of U based on max frequency
            #freq_max = 20000 # max freq of sine sweep in Hz
            #lengthU=int(freq_max*U.shape[0]*2/Fs)

            # calculate impulse response
            lengthU=400000
            H[:lengthU] = Y[:lengthU]/U[:lengthU]


            H[:int(50.*N/Fs)]=0
            #H[lengthU:]=0
            #H[400000:]=0

            # Apply window
            #M = int(N*MAX_F/Fs) # k of highest frequency of input signal
            #hann=np.zeros(H.shape)
            #hann[:M] = np.hanning(2*M)[M:]
            H_filtered=H

            # Create impulse response
            h_filtered = np.fft.irfft(H_filtered,n=N)

            # Adjust zooms to TOA if calculated priorily
            if self.U != '':
                max_time = np.max(self.U)
                zoom1 = [0,max_time*10]
                zoom2 = [3*TLAT/4,max_time*2]
                zoom3 = [9*TLAT/10,max_time*1.1]
            else:
                zoom1 = [0,float(T)/10.]
                zoom2 = [3*TLAT/4,float(T)/50.]
                zoom3 = [3*TLAT/4,float(T)/200.]

            # Show and save results
            plt.close('all')


            plt.figure(1)
            plt.plot(t,h_filtered[:len(t)]*1000),plt.axis('tight')
            plt.xlabel('t [s]'),plt.ylabel('h')
            plt.title('RIR step {0} channel {1}'.
                      format(step_number,channel_number))
            plt.savefig(name)

            plt.figure(2)
            plt.plot(f,abs(H))
            plt.xlabel('f (Hz)'),plt.ylabel('|H(f)|')
            plt.xlim([0,int(max(f)/2)])
            plt.title('Raw RIR and filter step {0} channel {1}'.
                      format(step_number,channel_number))
            plt.savefig(name+'_unfiltered')
            i = 1
            for zoom in [zoom1,zoom2,zoom3]:
                plt.figure(i+2)
                plt.plot(t,h_filtered[:len(t)]*1000)
                # plot vertical lines at estimated arrival times
                if self.U != '':
                    colors = ['red','green','black','orange']
                    j = 0
                    for x_TOA in self.U[:,step_number]:
                        plt.axvline(x=x_TOA,color=colors[j],linestyle='dashed',linewidth=1)
                        j=j+1
                # plot vertical line at latency time
                plt.axvline(x=TLAT,color='black',linestyle='-',linewidth=2)
                # plot parameters
                plt.xlim(zoom),plt.xlabel('t [s]'),plt.ylabel('h')
                plt.legend(['impulse response','wall 1','wall 2','wall 3','wall 4','latency'],'best')
                plt.title('RIR step {0} channel {1} zoom {2}'.
                      format(step_number,channel_number,i))
                plt.savefig(name+'_zoom'+str(i))
                i = i+1
            plt.show(block=False)
            #self.save_RIR(output_wav,h_filtered)
        return np.array([t,h_filtered]),np.array([f,H_filtered])
    def apply_filter(self,time,freq):
        t=time[0]
        h=time[1]
        f=freq[0]
        H=freq[1]
        nyq_rate = Fs/2
        width = .6/nyq_rate # filter with
        dip = 21.0 # desired attenuation

        #compute filter coefficints
        N_kaiser, beta = kaiserord(dip, width)
        # filter bandpass windows:
        peaks = []
        peaks_start = [1031,2387,3960,5208]
        peaks_number = [6,7,2,2]
        #peaks_start = [22680,52540,87180,114420]
        #step = 2390
        step=108
        for i,start in enumerate(peaks_start):
            for j in range(0,peaks_number[i]):
                peaks.append(start+j*step)
        #print(peaks)
        cutoff_nyq = []
        for p in peaks:
            cutoff_nyq.append((p-2)/nyq_rate)
            cutoff_nyq.append((p+2)/nyq_rate)
        cutoff_nyq.append(0.99)
        taps = firwin(N_kaiser, cutoff_nyq, window=('kaiser', beta))
        filtered_h = filtfilt(taps,[1.0],h)

        #fft on filtered data
        f_filt = np.fft.rfftfreq(N, d=1/Fs)
        H_filt = np.fft.rfft(filtered_h,n=N)

        #plot the Time Series comparison and the FFT comparison
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex = False, figsize=(12,15))

        ax1.set_title('Original and Notch Filtered Data')

        ax1.plot(t, h, color='r', alpha=1, label='Time Series Original')
        ax2.plot(t, filtered_h, color='r', alpha=1, label='Time Series Filtered')
        ax3.plot(f, H, color='b', alpha=1, label='Frequency Spectrum Original')
        ax4.plot(f, H_filt, color='b', alpha=1, label='Frequency Spectrum Notch Filtered')

        ax1.legend(frameon=False, fontsize=10)
        ax2.legend(frameon=False, fontsize=10)
        ax3.legend(frameon=False, fontsize=10)
        ax4.legend(frameon=False, fontsize=10)
        ax1.set_ylabel('Amplitude'); ax2.set_ylabel('Amplitude')
        ax1.set_xlabel('Time (sec)'); ax2.set_xlabel('Time (sec)')
        ax3.set_xlabel('Frequency (Hz)'); ax4.set_xlabel('Frequency (Hz)')
        ax3.set_ylabel('Energy'); ax4.set_ylabel('Energy')

        return np.array([t,h_filt]),np.array([f_filt,H_filt])
    def get_crosscorr(self):
        ''' Get cross correlation '''
        global MAX_LENGTH
        MAX_L = MAX_LENGTH
        corrs = []
        Fs,u = wavfile.read(self.input_wav)
        i=0
        for output_wav in self.output_wav_list:
            print("treating",output_wav)
            channel_number = int(output_wav[-5])
            # Read input files
            name=self.out_dir+str(channel_number)+'_CrossCorrelation'
            Fs2,y = wavfile.read(output_wav)
            if Fs2 != Fs:
                print("Warning: Sampling rate from input doens't match output:",
                      Fs,Fs2)
            Fs=float(Fs)
            # make signals go from 0 to 1 to avoid overflow:
            bits = 16 # nubmer of bits of wav file.
            u = u.astype(np.float32)/2**(bits-1)
            y = y.astype(np.float32)/2**(bits-1)
            # compute cross-correlation
            if u.shape[0]<MAX_LENGTH or y.shape[0]<MAX_LENGTH:
                MAX_L = np.min((u.shape[0],y.shape[0]))
            corr = np.correlate(u[:MAX_L],y[:MAX_L],"full")
            kcorr = np.linspace(-MAX_L+1,MAX_L-1,2*(MAX_L-1)+1)
            corrmax = kcorr[np.argmax(corr)]
            print('argmax: ',corrmax)
            print('time lag: ',corrmax/Fs)
            min
            plt.close(i)
            plt.figure(i)
            plt.plot(kcorr,corr)
            plt.axvline(corrmax,color='red',linestyle='dashed')
            plt.xlabel('k'),plt.ylabel('Ruy [k]')
            plt.title('Cross Correlation Ruy')
            plt.legend(['Ruy','Maximum at k = {0}'.format(corrmax)],loc=4)
            plt.show(block=False)
            plt.savefig(name)
            corrs.append(corr)
            i = i+1
        return corrs
    def save_RIR(self,y_file,h):
        ''' Saves RIR in binary file

        _Parameters_:
        y_file: input file for rir response (used for naming only)
        h: vector of impulse response

        _Returns_: Nothing
        '''
        wav_file = y_file.replace('audio','rir')
        wav_file = wav_file.split('/')[-1]
        wav_file = self.out_dir+wav_file
        wavfile.write(wav_file,Fs,h)
    def odometry(position_0,encoders):
        '''
        Calculates positions based on encoder measures and start position.

        _Parameters_:
        position_0: nparray of start position ([x0,y0,theta0]) in mm and radians rsp.
        theta0 is measured with respect to y axis, positive in the counter-clockwise
        sense.
        encoders: nparray of encoder measures at the N steps of the form
        [[left0, right0],[left1,right1],...]. size: 2*N

        _Returns_:
        positions: nparray of all resulting positons (x,y,theta)
        '''
        positions = []
        rs=[]
        dthetas=[]
        positions.append(position_0)
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
            # TODO: Check the sign of these for cos(theta2-theta1 negative)
            y2 = y1 + abs(r)*(np.cos(theta2-theta1)-1)
            x2 = x1 + r*(np.sin(theta2-theta1))
            positions.append(np.array([round(x2),round(y2),round(theta2,4)]))
            dthetas.append(dtheta)
            rs.append(r)
        return np.array(positions)
    def plot_odometry(odometry, real,fname):
        '''
        Plots the positions of the real positions and the positions calculated with
        odoemtry.

        _Parameters_:
            odometry:   nparray of odometry positions at N steps
                        ((x0',y0',[theta0']),(x1',y1'[theta1']),...) of size 2*N or 3*N
            real:   nparray of the real positions at N steps
                    ((x0,y0,[theta0]),(x1,y1,[theta1]),...) of size 2*N or 3*N.
            theta amy be given but won't be used in the plot.
        _Returns_: Nothing.
        '''
        plt.figure(5)
        plt.plot(pos[:,0],pos[:,1],label='odometry',marker='*'),
        plt.plot(real[:,0],real[:,1],label='real',marker='*')
        plt.xlabel('x [mm]'),plt.ylabel('y [mm]'),plt.legend()
        plt.title('Odometry precision study')
        plt.axis('equal'),plt.show(block=False)
        plt.savefig(out_dir+'odometry')

def get_parameters():
    ''' Get parameters from command line '''
    if len(sys.argv) < 3:
        print("Perform analysis: \n\nUsage: %s u_file.wav y_file.wav [y_file2.wav y_file3.wav ...] out_dir" % sys.argv[0])
        sys.exit(-1)
    u_file = sys.argv[1]
    y_files = sys.argv[2:-1]
    out_dir = sys.argv[-1]
    return u_file,y_files,out_dir
if __name__ == "__main__":
    # Odometry
    # calculate odometry positions.
    theta1 = np.pi
    encoders = np.array([[0.,0.],[-6825.,6699.],[-12957.,13392.]])
    position_0 = [round(pos_real[0][0]),round(pos_real[0][1]),round(theta1,4)]
    pos_odometry = odometry(position_0,encoders)
    # plot and save results
    pos_real = np.array([[2118,1975],[1829,1948],[1544,1935]])
    plot_odometry(pos_odometry, pos_real, out_dir+'odometry')

    # Plot Summary

