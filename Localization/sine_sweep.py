# -*- coding: utf-8 -*-
import numpy as np
from scipy.io import wavfile
import wave
import matplotlib.pyplot as plt

f1 = 100.     # Start frequency in [Hz]
f2 = 20000.  # End frequency in [Hz]
T  = 11.      # Pulse duration in [s]
fs = 44100.  # Sampling frequency in [Hz]
Ts = 1./fs   # Sampling period in [s]
N = np.floor(T/Ts)
n  = np.arange(0, N, dtype='float64')  # Sample index

om1 = 2*np.pi*f1
om2 = 2*np.pi*f2

# louder signal
bits=16
amp = (2**(bits-1)) # max value for signed int
fac = 0.8
x_exp = fac*amp*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))

t_win = 0.1
n_win = np.floor(t_win*fs)
win = np.hanning(t_win*fs)
x_exp[:n_win/2] *= win[:n_win/2]
x_exp[-n_win/2:] *= win[-n_win/2:]

plt.close(1)
plt.figure(1)
t = n/fs
t_detail = t_win*fs*3
plt.plot(t[:t_detail],x_exp[:t_detail])
plt.xlabel('t [s]'),plt.ylabel('y(t) [-]')
plt.title('Start of Sine Sweep')
plt.savefig('../Report/files/sweep_start.png')
plt.close(2)
plt.figure(2)
f = np.fft.rfftfreq(int(N), d=1/fs)
plt.plot(f,np.abs(np.fft.rfft(x_exp)))
plt.xlabel('f [Hz]'),plt.ylabel('| fft(y) |')
plt.title('FFT of Sine Sweep')
plt.savefig('../Report/files/sweep_fft.png')

plt.show(block=False)

x_exp_int = np.array(x_exp, dtype=np.int16)
#wavfile.write('latency_input/sound100.wav', fs, x_exp_int)
