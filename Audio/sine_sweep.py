
import numpy as np
from scipy.io import wavfile

f1 = 50.     # Start frequency in [Hz]
f2 = 22000.  # End frequency in [Hz]
T  = 11.      # Pulse duration in [s]
fs = 44100.  # Sampling frequency in [Hz]
Ts = 1./fs   # Sampling period in [s]
N = np.floor(T/Ts)
n  = np.arange(0, N, dtype='float64')  # Sample index

om1 = 2*np.pi*f1
om2 = 2*np.pi*f2

x_exp = 0.99*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))

t_win = 0.1
n_win = np.floor(t_win*fs)
win = np.hanning(t_win*fs)
x_exp[:n_win/2] *= win[:n_win/2]
x_exp[-n_win/2:] *= win[-n_win/2:]

import matplotlib.pyplot as plt
plt.figure()
plt.subplot(2,1,1)
plt.plot(x_exp)
plt.subplot(2,1,2)
plt.plot(np.log10(np.abs(np.fft.rfft(x_exp))))

wavfile.write('exp_sweep.wav', fs, x_exp)
