# -*- coding: utf-8 -*-
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

x_rand = np.random.randn(N)

#x_exp = 0.99*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))
bits=16
# louder signal
amplification = (2**(bits-1))
factor = 0.8
x_rand = factor*amplification*x_rand

x_rand_spectr = np.abs(np.fft.rfft(x_rand, n=int(N)))
x_rand_f = np.fft.rfftfreq(int(N),d=Ts)
n_bins = 100
values,bins=np.histogram(x_rand,n_bins)

plt.figure()
plt.subplot(1,2,1)
plt.plot(x_rand_f,x_rand_spectr)
plt.xlabel('Frequency [Hz]'),plt.ylabel('Spectral density')
plt.title('Spectral Density of Random Signal')
plt.subplot(1,2,2)
plt.plot(bins[:-1],values)
plt.xlabel('Lower limit of bin'),plt.ylabel('Number of values in bin')
plt.title('Histogram of random signal')
plt.show(block=False)

x_rand_int = np.array(x_rand, dtype=np.int16)
wavfile.write('latency_input/random1.wav', fs, x_rand_int)
