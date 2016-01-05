# -*- coding: utf-8 -*-
'''
Analysis.py module
==============

Contains functions to analyze room impulse response, odometry measurements
and visual localization.

'''
import sys
import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt
from scipy.signal import firwin, kaiserord, filtfilt, lfilter
import calibrate as calib

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
WIDTH_ROOM = 7078 #in mm
HEIGHT_ROOM = 7311 #in mm
N_WALLS = 4
C = 343200 # speed of sound at 20C, dry air, in mm/s
# CROSS CORRELATION
MAX_LENGTH = Fs*2
TLAT = 0.1454 # Latency time, found with 660mm distance test

# res2:
MARGIN = np.array([2000,1000],dtype=np.float) #MARGIN from leftmost and downlost ref point to reference (in mm)
PTS_BASIS = np.array(([2275,3769],[3128,3713])) #position of first and second reference points from wall (in mm)
# res1:
#MARGIN = np.array([1000,1000],dtype=np.float) #MARGIN from leftmost and downlost ref point to reference (in mm)
#PTS_BASIS = np.array(([2500,3000],[4000,1500])) #position of first and second reference points from wall (in mm)

class Analysis():
    def __init__(self,out_dir,input_wav='', output_wav_list='', output_enc='',
                 output_real='',output_vis='',output_cam=''):
        '''
        _Parameters_:
        out_dir: folder where results should be saved
        input_wav: audio input file with path (most likely sine sweep)
        output_wav_list: list of audio output files
        output_end: encoder output file
        output_real: real positions output file
        output_vis: visual localization output file
        output_cam: camera centers output file

        '''
        if out_dir[-1]!='/':
            out_dir = out_dir+'/'
        self.out_dir = out_dir
        self.input_wav = input_wav
        self.output_wav_list = output_wav_list
        self.output_enc = output_enc
        self.output_real = output_real
        self.output_vis = output_vis
        self.output_cam = output_cam
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
            name=self.out_dir+str(step_number)+'_'+str(channel_number)+'_NEW_filtY'
            Fs2,y = wavfile.read(output_wav)
            if Fs2 != Fs:
                print("Warning: Sampling rate from input doens't match output:",
                      Fs,Fs2)
            # reshape to one-channel array (can be removed once Audio.py is
            # fixed)
            y = y.reshape((-1))
            Fs=float(Fs)
            # Deconvolute signals
            t = np.linspace(0,T,T*Fs) # time vector
            N=2*max(len(u),len(y))
            f = np.fft.rfftfreq(N, d=1/Fs)
            Y = np.fft.rfft(y,n=N) # automatically adds 0s until N

            U = np.fft.rfft(u,n=N)
            # apply filter to Y to filter out harmonic frequencies
            __,[f,Y] = self.apply_filter([t,y],[f,Y],'Y')
            H = np.zeros((max(len(u),len(y))+1),dtype=np.complex)
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
            #__,[f,H] = self.apply_filter([t,y],[f,H],'H')
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
    def apply_filter(self,time,freq,method='Y'):
        '''
        Filters out required frequencies from frequency spectrum of signal.

        _Parameters_:
            time:   [t,h] nparray of time vector and corresponding time response,
                    as returned by get_RIR(). Only used in method 'kaiser'
            freq:   [f,H] nparray of frequency vector and corresponding FFT, as
                    returned by get_RIR(). Indirectly filtered with 'kaiser',
                    directly filtered with other methods.

            method: if set to 'kaiser', the Kaiser method from
                    http://nbviewer.ipython.org/github/LORD-MicroStrain/SensorCloud/blob/master/MathEngine/Example%20Notebooks/LORD%20Notch%20Filter.ipynb
                    ATTENTION: never tested completely because it always led to
                    crash because of overloaded software memory.
                    When set to anything other than 'kaiser', the desired
                    frequencies are simply set to 0 and the parameter method is
                    only used for naming the output files.(default: Y)
        '''

        t=time[0]
        h=time[1]
        f=freq[0]
        H=freq[1]

        # filter bandpass windows:
        peaks = []
        #peaks_start = [49]
        #peaks_number = [50]
        peaks_start = [1031,2387,3960,5208]
        peaks_number = [6,7,2,2]
        step=108
        width_notch = 4 # half width of notch filter
        for i,start in enumerate(peaks_start):
            for j in range(0,peaks_number[i]):
                peaks.append(start+j*step)
        if method == 'kaiser':
            cutoff_nyq = []
            for p in peaks:
                cutoff_nyq.append((p-width_notch)/nyq_rate)
                cutoff_nyq.append((p+width_notch)/nyq_rate)
            cutoff_nyq.append(0.99)

            nyq_rate = Fs/2
            width = .6/nyq_rate # filter with
            dip = 21.0 # desired attenuation

            #compute filter coefficints
            N_kaiser, beta = kaiserord(dip, width)
            taps = firwin(N_kaiser, cutoff_nyq, window=('kaiser', beta))
            h_filt = lfilter(taps,[1.0],h)
            # h_filt = filtfilt(taps,[1.0],h) RAN OUT OF MEMORY!
            f_filt = np.fft.rfftfreq(N, d=1/Fs)
            H_filt = np.fft.rfft(filtered_h,n=N)
        elif method!='kaiser':
            # set points around peaks to zero.
            H_filt = H.copy()
            for p in peaks:
                notch = (f<p+width_notch)&(f>p-width_notch)
                H_filt[notch]=0
            f_filt = f
            h_filt = h

        #plot the Time Series comparison and the FFT comparison

        #fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex = False, figsize=(12,15))
        fig, (ax3, ax4) = plt.subplots(2, 1, sharex = False, figsize=(6,8))

        #ax1.set_title('Original and Notch Filtered Data')

        #ax1.plot(t, h[:len(t)], color='r', alpha=1, label='Time Series Original')
        #ax2.plot(t, h_filt[:len(t)], color='r', alpha=1, label='Time Series Filtered')
        ax3.plot(f, abs(H), color='b', alpha=1, label='Frequency Spectrum Original')
        leg=''
        for p in peaks:
            if leg=='':
                leg='Notch Filter Locations'
                ax3.axvline(x=p,color='red',linestyle='dashed',linewidth=1,label=leg)
            else:
                ax3.axvline(x=p,color='red',linestyle='dashed',linewidth=1)
        ax4.plot(f, abs(H_filt), color='b', alpha=1, label='Frequency Spectrum Notch Filtered')
        ax3.ticklabel_format(axis='y', style='sci', scilimits=(-2,4))
        ax4.ticklabel_format(axis='y', style='sci', scilimits=(-2,4))
        ax3.set_xlim([0,max(peaks)*11/10])
        ax4.set_xlim([0,max(peaks)*11/10])
        #ax1.legend(frameon=False, fontsize=10)
        #ax2.legend(frameon=False, fontsize=10)
        ax3.legend(frameon=False, fontsize=10)
        ax4.legend(frameon=False, fontsize=10)
        #ax1.set_ylabel('Amplitude'); ax2.set_ylabel('Amplitude')
        #ax1.set_xlabel('Time (sec)'); ax2.set_xlabel('Time (sec)')
        ax3.set_xlabel('f [Hz]'); ax4.set_xlabel('f [Hz]')
        plt.show(block=False)
        plt.savefig('../Report/files/Notch_test'+method+'.png')

        return np.array([t,h_filt]),np.array([f_filt,H_filt])
    def get_crosscorr(self):
        ''' Get cross correlation between self.input_files and
        self.output_files'''
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
    def odometry(self,position_0,encoders):
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
        pos_old = position_0
        print(pos_old)
        positions.append(pos_old)
        enc_old=encoders[0]
        for i in range(len(encoders)-1):
            enc = encoders[i+1]-encoders[i]

            x1 = pos_old[0]
            y1 = pos_old[1]
            theta1 = pos_old[2]
            # conversion in mm
            l_left = enc[0]/N_tot*2*np.pi*R_wheels
            print(l_left)
            l_right = -enc[1]/N_tot*2*np.pi*R_wheels
            print(l_right)
            dtheta=(l_left-l_right)/D
            theta2 = dtheta+theta1
            r=D/2*(l_left+l_right)/(l_left-l_right)

            # calculate new positions
            y2 = y1 + r*(np.sin(theta1)-np.sin(theta2))
            x2 = x1 + r*(np.sin(theta2-theta1))
            # res 2:
            pos_old=np.array([round(x2),round(y2),round(theta2,4)])
            positions.append(pos_old)
            # res 1:

            #pos_old=np.array(calib.change_ref_to_wall(PTS_BASIS,MARGIN,
            #    np.array([round(x2),round(y2),round(theta2,4)]).reshape(1,3)))[0]
            #positions.append(pos_old)

            print(pos_old)
            dthetas.append(dtheta)
            rs.append(r)
        return np.array(positions)
    def plot_odometry(self,odometry, real,fname):
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
    def plot_geometry(self,fname=''):
        ''' Reads results from files of specified output file names (if not empty)
        and plots everything in the wall reference frame i
        _Parameters_:
            fname:   file name (with path) where resulting image is saved
        _Returns_:
            nothing
        '''
        plt.figure()
        legend_str=[]

        # plot line between frist two reference points
        if PTS_BASIS != '':
            ax = plt.plot(PTS_BASIS[:,0],PTS_BASIS[:,1],color='k',linestyle='-')
            legend_str.append('ref basis')
        # plot reference coordinate system
        origin=calib.change_ref_to_wall(PTS_BASIS,MARGIN,np.matrix([0,0,0]))
        xaxis=calib.change_ref_to_wall(PTS_BASIS,MARGIN,np.matrix([MARGIN[1],0,0]))
        yaxis=calib.change_ref_to_wall(PTS_BASIS,MARGIN,np.matrix([0,MARGIN[1],0]))
        plt.plot([yaxis[0,0],origin[0,0],xaxis[0,0]],[yaxis[0,1],origin[0,1],xaxis[0,1]],color='b',linestyle=':')
        legend_str.append('ref origin')
        # plot wall
        x=[0,WIDTH_ROOM,WIDTH_ROOM,0,0]
        y=[0,0,HEIGHT_ROOM,HEIGHT_ROOM,0]
        plt.plot(x,y,linestyle='dashed',color='k')
        legend_str.append('walls')
        # plot robot positions
        if self.output_real!='':
            real_array = np.loadtxt(self.output_real,dtype=np.float32)
            # for results of 22.11. (not yet in wall reference)
            #array_wall = []
            #for c in real_array:
            #    res=calib.change_ref_to_wall(PTS_BASIS,MARGIN,c.reshape(1,3))
            #    array_wall.append(res)
            #real_array = np.array(array_wall).reshape(3,3)
            plt.plot(real_array[:,0],real_array[:,1],marker='x',color='g')
            legend_str.append('real')
        # plot visual positions
        if self.output_vis!='':
            obj_array = np.loadtxt(self.output_vis,dtype=np.float32)
            if len(obj_array.shape) > 1:
                plt.plot(obj_array[:,0],obj_array[:,1],marker='x',color='r')
            else:
                # for results of 22.11. (not yet in wall reference)
                #obj_array = calib.change_ref_to_wall(PTS_BASIS,MARGIN,obj_array.reshape(1,3))
                #plt.plot(obj_array[0,0],obj_array[0,1],marker='x',color='r')

                plt.plot(obj_array[0],obj_array[1],marker='x',color='r')

            legend_str.append('visual')
        # plot odometry positions
        if self.output_enc!='':
            enc_array = np.loadtxt(self.output_enc,dtype=np.float32)
            array_wall = []
            pos_0 = [round(real_array[0,0]),round(real_array[0,1]),round(np.pi/4,4)]
            odo_array = self.odometry(pos_0,enc_array)
            plt.plot(odo_array[:,0],odo_array[:,1],marker='x',color='k')
            legend_str.append('odometry')
        # plot camera positions
        if self.output_cam!='':
            cam_centers = np.loadtxt(self.output_cam,dtype=np.float32)
            cam_centers_wall = []
            for c in cam_centers:
                res=calib.change_ref_to_wall(PTS_BASIS,MARGIN,c[1:].reshape(1,3))
                c[1:]=res[0]
                cam_centers_wall.append(c)
            cam_centers = np.array(cam_centers_wall)
            colors={139:'blue',141:'red',143:'green',145:'orange'}
            for i in range(0,cam_centers.shape[0]):
                plt.plot(cam_centers[i,1],cam_centers[i,2],marker='o',linestyle='',color=colors[cam_centers[i,0]])
                #legend_str.append(str(int(cam_centers[i,0])))
        plt.legend(legend_str,'best')
        # plot properties
        plt.grid(True)
        plt.axes().set_aspect('equal','datalim')
        plt.xlabel('x [mm]'),plt.ylabel('y [mm]')
        plt.title('Experimental Results in Room Reference Frame \n')
        plt.show(block=False)
        if fname!='':
            plt.savefig(fname)

def get_parameters():
    ''' Get parameters from command line '''
    if len(sys.argv) < 3:
        print("Perform analysis: \n\nUsage: %s u_file.wav y_file.wav [y_file2.wav y_file3.wav ...] out_dir" % sys.argv[0])
        sys.exit(-1)
    u_file = sys.argv[1]
    y_files = sys.argv[2:-1]
    out_dir = sys.argv[-1]
    return u_file,y_files,out_dir

