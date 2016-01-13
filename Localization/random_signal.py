# -*- coding: utf-8 -*-
##@package random_signal
# Random signal
# ============
# Creates randomn signal (white noise) and saves it in .wav file
#
import numpy as np
from scipy.io import wavfile
import wave
import matplotlib.pyplot as plt

# MY CODE

T  = 1.      # Pulse duration in [s]
fs = 44100.  # Sampling frequency in [Hz]
Ts = 1./fs   # Sampling period in [s]
N = np.floor(T/Ts)
n  = np.arange(0, N, dtype='float64')  # Sample index
bits=16
amplification = (2**(bits-1))
factor = 0.8

M = 100

x_rand_sum = np.zeros((N/2+1))
x_rand_sum_amp = np.zeros((N/2+1))
for i in range(0,M):
    x_rand = factor*amplification*np.random.randn(N)
    x_rand_spectr = np.abs(np.fft.rfft(x_rand, n=int(N)))
    x_rand_sum += np.power(x_rand_spectr,2)/(N*np.std(x_rand)**2)
x_rand_sum = x_rand_sum/M

x_rand_f = np.fft.rfftfreq(int(N),d=Ts)
n_bins = 100
values,bins=np.histogram(x_rand,n_bins)

# make plots
plt.close(1)
plt.figure(1)
plt.plot(x_rand_f,x_rand_spectr,':')
plt.xlabel('f [Hz]')
plt.title('Magnitude of FFT of Random Signal')
plt.savefig('../Report/files/random_fft.png')

plt.close(2)
plt.figure(2)
plt.plot(bins[:-1],values)
plt.xlabel('Lower limit of bin'),plt.ylabel('Number of elements in bin')
plt.title('Histogram of Random Signal')
plt.savefig('../Report/files/random_hist.png')

plt.close(3)
plt.figure(3)
plt.plot(x_rand_f,x_rand_sum,':')
plt.ylim([0,2])
plt.xlabel('f [Hz]')
plt.title('Averaged and Normalized DFT Square Magnitude with M = '+str(M))
plt.savefig('../Report/files/random_M'+str(M)+'.png')

plt.show(block=False)

# save wav file
x_rand_amp = factor*amplification*x_rand
x_rand_int = np.array(x_rand, dtype=np.int16)
#wavfile.write('latency_input/random1.wav', fs, x_rand_int)
