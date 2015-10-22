
import numpy as np
from scipy.io import wavfile

f1 = 50.     # Start frequency in [Hz]
f2 = 22000.  # End frequency in [Hz]
T  = 11.      # Pulse duration in [s]
fs = 44100.  # Sampling frequency in [Hz]
Ts = 1./fs   # Sampling period in [s]
N = np.floor(T/Ts)
n  = np.arange(0, N, dtype='int16')  # Sample index

om1 = 2*np.pi*f1
om2 = 2*np.pi*f2

x_exp = 0.99*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))
wavfile.write('sound.wav', fs, x_exp)
