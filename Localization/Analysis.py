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
MIN_F = 50# minimum of sine sweep
MAX_F = 20000  # maximum of sine sweep
Fs = 44100. # sampling rate
T = 10 # seconds of interest for room impulse response
WIDTH_ROOM = 7078 # in mm
HEIGHT_ROOM = 7311 # in mm
HEIGHT_CEILING = 1494 # in mm, measured from microphones
N_WALLS = 5
C = 343200 # speed of sound at 20C, dry air, in mm/s
# CROSS CORRELATION
MAX_LENGTH = Fs*2
TLAT = 0.1454 # Latency time, found with 660mm distance test

# res2:
MARGIN = np.array([2000,2000],dtype=np.float) #MARGIN from leftmost and downlost ref point to reference (in mm)
PTS_BASIS = np.array(([2746,3066],[3506,2708])) #position of first and second reference points from wall (in mm)
# res2:
#THETA_0=round(np.pi/4,4)
THETA_0=round(3*np.pi/2,4)
# res1:
#MARGIN = np.array([1000,1000],dtype=np.float) #MARGIN from leftmost and downlost ref point to reference (in mm)
#PTS_BASIS = np.array(([2500,3000],[4000,1500])) #position of first and second reference points from wall (in mm)

class Analysis():
    def __init__(self,out_dir,input_wav='', output_wav_list='', output_enc='',output_mov='',
                 output_real='',output_vis_fix='',output_vis_free='',output_cam='',output_camreal=''):
        '''
        _Parameters_:
        out_dir: folder where results should be saved
        input_wav: audio input file with path (most likely sine sweep)
        output_wav_list: list of audio output files
        output_enc: encoder output file
        output_mov: expected robot movement file
        output_real: real positions output file
        output_vis: visual localization output file
        output_cam: camera centers output file
        output_camreal: real camera centers file

        '''
        if out_dir[-1]!='/':
            out_dir = out_dir+'/'
        self.out_dir = out_dir
        self.input_wav = input_wav
        self.output_wav_list = output_wav_list
        self.output_enc = output_enc
        self.output_mov = output_mov
        self.output_real = output_real
        self.output_vis_fix = output_vis_fix
        self.output_vis_free = output_vis_free
        self.output_cam = output_cam
        self.output_camreal = output_camreal
        self.U=''
        self.y=[]
        self.u=''
        self.cam=''
        self.camwall=''
        self.camreal=''
        self.real=''
        self.enc=''
        self.odo=''
        self.mov=''
        self.vis_free=''
        self.vis_fix=''
    def read_file(self,fname):
        if type(fname)==str:
            if fname[-3:]=='wav': # input wav file
                Fs2,u= wavfile.read(fname)
                return u
            else: # all .txt output files
                return np.loadtxt(fname,dtype=np.float32)
        if type(fname)==list: # output wav file list
            y_arr=[]
            for output_wav in fname:
                y_dic = dict()
                fname = output_wav.partition('/')
                fname = [a for a in fname if a!= '']
                step_number = int(fname[-1][0])
                channel_number = int(output_wav[-5])
                Fs,y = wavfile.read(output_wav)
                y_dic['channel']=channel_number
                y_dic['step']=step_number
                y_dic['y']=y
                y_dic['Fs']=Fs
                y_arr.append(y_dic)
            return y_arr
    def read_files(self):
        ''' reads the files if they are specified'''
        if self.input_wav!='':
            self.u=self.read_file(self.input_wav)
        if self.output_wav_list!='':
            self.y=self.read_file(self.output_wav_list)
        if self.output_cam!='':
            self.cam=self.read_file(self.output_cam)
        if self.output_camreal!='':
            self.camreal=self.read_file(self.output_camreal)
        if self.output_real!='':
            self.real=self.read_file(self.output_real)
        if self.output_vis_fix!='':
            self.vis_fix=self.read_file(self.output_vis_fix)
        if self.output_vis_free!='':
            self.vis_free=self.read_file(self.output_vis_free)
        if self.output_enc!='':
            self.enc=self.read_file(self.output_enc)
        if self.output_mov!='':
            self.mov=self.read_file(self.output_mov)
    def get_TOA(self):
        '''
        Calculate matrix of estimated time of arrival based on distances
        to wall, calculated from real positions and room width and height.

        _Returns_:
            np.Array U of size K*N_steps, where N_steps is number of steps and
            K is number of walls. Therefore, the rows correspond the response
            times from the K walls and the columns correspond to steps.
        '''
        U = np.zeros((N_WALLS,self.real.shape[0]))
        for i in range(0,self.real.shape[0]):
            U[0,i]=(HEIGHT_ROOM-self.real[i,1])
            U[1,i]=(WIDTH_ROOM-self.real[i,0])
            U[2,i]=self.real[i,1]
            U[3,i]=self.real[i,0]
            U[4,i]=HEIGHT_CEILING
        self.U = U*2/C+TLAT
        return self.U
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
        for output_wav in self.y:
            y = output_wav['y']
            step_number=output_wav['step']
            channel_number=output_wav['channel']
            name=self.out_dir+str(step_number)+'_'+str(channel_number)+'_filt'
            Fs=float(output_wav['Fs'])
            # Deconvolute signals
            t = np.linspace(0,T,T*Fs) # time vector
            N=2*max(len(self.u),len(y))
            f = np.fft.rfftfreq(N, d=1/Fs)
            Y = np.fft.rfft(y,n=N) # automatically adds 0s until N
            U = np.fft.rfft(self.u,n=N)

            #filter Y and U
            hann=np.zeros(Y.shape)
            M=20000
            hann[:M] = np.hanning(2*M)[M:]
            plt.figure()
            plt.plot(f,[Y,hann])

            # apply filter to Y to filter out harmonic frequencies
            #__,[f,Y] = self.apply_filter([t,y],[f,Y],'Y')

            # calculate impulse response
            H = np.zeros(N/2+1,dtype=np.complex)
            # delete response after max frequency
            lengthU=9./10.*MAX_F*N/Fs
            H[:lengthU] = Y[:lengthU]/U[:lengthU]
            # delete response up to min frequency
            H[:int(MIN_F*N/Fs)]=0

            # Apply window
            #hann=np.zeros(H.shape)
            #hann[:M] = np.hanning(2*M)[M:]

            #__,[f,H_filtered] = self.apply_filter([t,y],[f,H],'H')
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
            plt.xlim([0,int(max(abs(f))/2)])
            plt.title('Raw RIR and filter step {0} channel {1}'.
                      format(step_number,channel_number))
            #plt.savefig(name+'_unfiltered')
            i = 1
            for zoom in [zoom2,zoom3]:
                legend_str=[]
                plt.figure(i+2)
                plt.plot(t,h_filtered[:len(t)]*1000)
                legend_str.append('impulse response')
                # plot vertical lines at estimated arrival times
                if self.U != '':
                    colors = ['red','green','black','orange','grey']
                    j = 0
                    for x_TOA in self.U[:,step_number]:
                        plt.axvline(x=x_TOA,color=colors[j],linestyle='dashed',linewidth=1)
                        j=j+1
                    legend_str.append(['wall 1','wall 2','wall 3','wall 4','ceiling'])
                # plot vertical line at latency time
                plt.axvline(x=TLAT,color='black',linestyle='-',linewidth=2)
                legend_str.append('latency')
                # plot parameters
                plt.xlim(zoom),plt.xlabel('t [s]'),plt.ylabel('h')
                plt.legend(legend_str)
                plt.legend(['impulse response','wall 1','wall 2','wall 3','wall 4','ceiling','latency'],'best')
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
        plt.savefig('self.out_dir'+'Notch_test'+method+'.png')

        return np.array([t,h_filt]),np.array([f_filt,H_filt])
    def get_crosscorr(self):
        ''' Get cross correlation between self.input_files and
        self.output_files'''
        global MAX_LENGTH
        MAX_L = MAX_LENGTH
        corrs = []
        i=0
        for output_wav in self.y:
            y = output_wav['y']
            step_number=output_wav['step']
            channel_number=output_wav['channel']
            name=self.out_dir+str(step_number)+'_'+str(channel_number)+'_CrossCorr'
            Fs=float(output_wav['Fs'])
            # Read input files
            # make signals go from 0 to 1 to avoid overflow:
            bits = 16 # nubmer of bits of wav file.
            u = self.u.astype(np.float32)/2**(bits-1)
            y = y.astype(np.float32)/2**(bits-1)
            # compute cross-correlation
            if u.shape[0]<MAX_LENGTH or y.shape[0]<MAX_LENGTH:
                MAX_L = np.min((u.shape[0],y.shape[0]))
            corr = np.correlate(u[:MAX_L],y[:MAX_L],"full")
            kcorr = np.linspace(-MAX_L+1,MAX_L-1,2*(MAX_L-1)+1)
            corrmax = kcorr[np.argmax(corr)]
            print('argmax: ',corrmax)
            print('time lag: ',corrmax/Fs)
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
            l_right = -enc[1]/N_tot*2*np.pi*R_wheels
            dtheta=(l_left-l_right)/D
            theta2 = dtheta+theta1
            r=D/2*(l_left+l_right)/(l_left-l_right)

            # calculate new positions
            y2 = y1 + r*(np.sin(theta2)-np.sin(theta1))
            x2 = x1 + r*(np.cos(theta2)-np.cos(theta1))
            pos_old=np.array([round(x2),round(y2),round(theta2,4)])
            positions.append(pos_old)

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
    def plot_geometry(self,fname='',zoom=False):
        '''
        Plots all results read by read_files() in the wall reference frame.
        Run read_files() before running this function.

        _Parameters_:
            [fname:]    file name where resulting image is saved (within self.out_dir)
                        no output saved if not specified or set to '' (default '')
            [zoom:]     weather to zoom to positions or not. (default False)
        _Returns_:
            nothing
        '''
        if zoom:
            plt.figure(figsize=(10,5))
        else:
            plt.figure(figsize=(9,8))
        legend_str=[]

        if not zoom:
            # plot line between frist two reference points
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
            plt.plot(self.real[:,0],self.real[:,1],marker='o',color='g')
            legend_str.append('real')
        # plot visual positions
        if self.output_vis_fix!='':
            if len(self.vis_fix.shape) > 1:
                plt.plot(self.vis_fix[:,0],self.vis_fix[:,1],marker='x',color='r')
            else:
                plt.plot(self.vis_fix[0],self.vis_fix[1],marker='x',color='r')
            if zoom:
                error = np.linalg.norm(self.real[:,:2]-self.vis_fix[:,:2])/self.real.shape[0]
                legend_str.append('visual fixed RMS-error = {0:3.2f}'.format(error))
            else:
                legend_str.append('visual fixed')
        if self.output_vis_free!='':
            if len(self.vis_free.shape) > 1:
                plt.plot(self.vis_free[:,0],self.vis_free[:,1],marker='x',color='grey')
            else:
                plt.plot(self.vis_free[0],self.vis_free[1],marker='x',color='grey')
            if zoom:
                error = np.linalg.norm(self.real[:,:2]-self.vis_free[:,:2])/self.real.shape[0]
                legend_str.append('visual free RMS-error = {0:3.2f}'.format(error))
            else:
                legend_str.append('visual free')
        # plot odometry positions
        if self.output_enc!='':
            pos_0 = [round(self.real[0,0]),round(self.real[0,1]),THETA_0]
            self.odo = self.odometry(pos_0,self.enc)
            plt.plot(self.odo[:,0],self.odo[:,1],marker='x',color='k')
            if zoom:
                error = np.linalg.norm(self.real[:,:2]-self.odo[:,:2])/self.real.shape[0]
                legend_str.append('odometry RMS-error = {0:3.2f}'.format(error))
            else:
                legend_str.append('odometry')
        # plot expected robot movement
        if self.output_mov!='':
            plt.plot(self.mov[:,0],self.mov[:,1],marker='o',markersize=2,color='k')
            error = np.linalg.norm(self.real[:,:2]-self.mov[:,:2])/self.real.shape[0]
            legend_str.append('expected RMS-error = {0:3.2f}'.format(error))
        # plot camera positions
        if self.output_cam!='' and not zoom:
            cam_wall = []
            # first time: change to wall reference frame.
            if self.camwall=='':
                for c in self.cam:
                    res=calib.change_ref_to_wall(PTS_BASIS,MARGIN,c[1:].reshape(1,3))
                    c[1:]=res[0]
                    cam_wall.append(c)
                self.camwall = np.array(cam_wall)
            colors={139:'blue',141:'red',143:'green',145:'orange'}
            for i in range(0,self.camwall.shape[0]):
                plt.plot(self.camwall[i,1],self.camwall[i,2],marker='x',linestyle='',color=colors[self.camwall[i,0]])
                #legend_str.append(str(int(self.camwall[i,0])))
        if self.output_camreal!='' and not zoom:
            colors={139:'blue',141:'red',143:'green',145:'orange'}
            for i in range(0,self.camreal.shape[0]):
                plt.plot(self.camreal[i,1],self.camreal[i,2],marker='o',linestyle='',color=colors[self.camreal[i,0]])
                #legend_str.append(str(int(self.camreal[i,0])))
        plt.legend(legend_str,'best')
        # plot properties
        plt.grid(True)
        plt.xlabel('x [mm]'),plt.ylabel('y [mm]')
        if zoom:
            #plt.xlim([2000,5000])
            #plt.ylim([2000,5000])
            ax=plt.gca()
            ax.relim()
            ax.autoscale_view()
            plt.axes().set_aspect('equal')
            plt.title('Experimental Results - Robot Positions\n')
        else:
            plt.axes().set_aspect('equal')
            plt.xlim([-1000,8000])
            plt.ylim([-1000,8000])
            plt.title('Experimental Results in Room Reference Frame \n')
        plt.show(block=False)
        if fname!='':
            plt.savefig(self.out_dir+fname)

def get_parameters():
    ''' Get parameters from command line '''
    if len(sys.argv) < 3:
        print("Perform analysis: \n\nUsage: %s u_file.wav y_file.wav [y_file2.wav y_file3.wav ...] out_dir" % sys.argv[0])
        sys.exit(-1)
    u_file = sys.argv[1]
    y_files = sys.argv[2:-1]
    out_dir = sys.argv[-1]
    return u_file,y_files,out_dir
def get_crosscor2(t,y1,y2):
    sig=50000
    plot(t[:sig],y1[:sig])

