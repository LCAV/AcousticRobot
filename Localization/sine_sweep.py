# -*- coding: utf-8 -*-
import numpy as np
from scipy.io import wavfile
import wave
import matplotlib.pyplot as plt


# FROM INTERNET
class SoundFile:
    def  __init__(self, signal,wavname):
        self.file = wave.open(wavname, 'wb')
        self.signal = signal
        self.sr = 44100

    def write(self,nchannels=1,sampwidth=2,framerate=44100,duration=4):
        self.file.setparams((nchannels, sampwidth, self.sr, framerate*duration, 'NONE', 'noncompressed'))
        self.file.writeframes(self.signal)
        self.file.close()

# let's prepare
duration = 4 # seconds
samplerate = 44100 # Hz
samples = duration*samplerate
frequency = 440 # Hz
period = samplerate / float(frequency) # in sample points
omega = np.pi * 2 / period

xaxis = np.arange(int(period),dtype = np.float) * omega
ydata = 16384 * np.sin(xaxis) # maximum volume

signal = np.resize(ydata, (samples,))

ssignal = ''
for i in range(len(signal)):
    ssignal += wave.struct.pack('h',signal[i]) # transform to binary

#f = SoundFile(ssignal,'audio/test.wav')
#f.write()

print('example file written')

# MY CODE

f1 = 50.     # Start frequency in [Hz]
f2 = 22000.  # End frequency in [Hz]
T  = 11.      # Pulse duration in [s]
fs = 44100.  # Sampling frequency in [Hz]
Ts = 1./fs   # Sampling period in [s]
N = np.floor(T/Ts)
n  = np.arange(0, N, dtype='float32')  # Sample index

om1 = 2*np.pi*f1
om2 = 2*np.pi*f2

#x_exp = 0.99*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))

# louder signal
bits = 16
x_exp = (2**(bits-1))*np.sin(om1*N*Ts / np.log(om2/om1) * (np.exp(n/N*np.log(om2/om1)) - 1))

t_win = 0.1
n_win = np.floor(t_win*fs)
win = np.hanning(t_win*fs)
x_exp[:n_win/2] *= win[:n_win/2]
x_exp[-n_win/2:] *= win[-n_win/2:]

plt.figure()
plt.subplot(2,1,1)
plt.plot(x_exp)
plt.subplot(2,1,2)
plt.plot(np.log10(np.abs(np.fft.rfft(x_exp))))

x_exp_int = np.array(x_exp, dtype=np.int16)
# this creates a float64 file that pyaudio cannot read...
wavfile.write('audio/exp_sweep_32.wav', fs, x_exp_int)
'''
# therefore this is added...
nchannels=1
sampwidth=2 # in bytes


x_exp_binary = ''
for i in range(int(N)):
     x_exp_binary += wave.struct.pack('h',x_exp[i]) # transform to binary
f = SoundFile(x_exp_binary,'audio/exp_sweep_louder.wav')
f.write(nchannels,sampwidth,fs,T)
print("my file written")



writer = wave.Wave_write('new_wave')
writer.setnchannels(1)
writer.setframerate(44100)
writer.setnframes(x_exp.shape[0])
writer.setsampwidth(2)
writer.writeframes(x_exp)

'''
