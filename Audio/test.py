# -*- coding: utf-8 -*-
""" Play and Record Test """
from __future__ import print_function, division
import pyaudio
import wave
import sys
import numpy as np

def get_device_index(p):
    ''' Lets user manually choose the device used for audio input and output.'''
    for i in range(p.get_device_count()):
        print("Index ",i,":\n",p.get_device_info_by_index(i))

    index = int(input("\nEnter index of Audio device to be used: \n"))
    return index

# input information
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS=1
RATE=44100
TYPE=None #order of the data for multidimensional arrays.

if len(sys.argv) < 2:
     print("Plays a wave file.\n\nUsage: %s input.wav output.wav" % sys.argv[0])
     sys.exit(-1)

FIN = sys.argv[1]
FOUT = sys.argv[2]
FOUT = FOUT.replace(".wav","")

wf_in = wave.open(FIN,'rb')
# create managing PyAudio instances
p_out = pyaudio.PyAudio()
p_in = pyaudio.PyAudio()

INDEX = get_device_index(p_in)

# initialize streams
stream_out = p_out.open(output_device_index=INDEX,
                        format=p_out.get_format_from_width(wf_in.getsampwidth()),
                        channels=wf_in.getnchannels(),
                        rate=wf_in.getframerate(),
                        output=True)
stream_in = p_in.open(input_device_index=INDEX,
                      format=FORMAT,
                      channels=CHANNELS,
                      rate=RATE,
                      input=True,
                      frames_per_buffer=CHUNK)
# initialize frames (to store recorded data in)
frames = []
frames_decoded = []

# read data
data = wf_in.readframes(CHUNK)
while data != '':
    # play
    stream_out.write(data)
    data = wf_in.readframes(CHUNK)
    # record
    data_in = stream_in.read(CHUNK)
    frames.append(data_in)
    frames_decoded.append(np.fromstring(data_in,np.int16))
frames_decoded=np.matrix(frames_decoded)
# store number of buffers (circa Time * RATE / CHUNKS)
BUFFERS = frames_decoded.shape[0]

# separate frames to channels
frames_ordered=frames_decoded.reshape(CHANNELS,BUFFERS*CHUNKS)

#number of buffers*CHUNK)
wf_in.close()

# stop streams
stream_out.stop_stream()
stream_out.close()
stream_in.stop_stream()
stream_in.close()

# close PyAudio
p_in.terminate()

# save recorded data in audio file
for i in range(CHANNELS):
    frames_channel = frames_ordered[:,i] # one array of int16 values
    frames_channel = frames_channel.reshape(BUFFERS,CHUNKS) #put into original shape
    # recover data format (in one string per buffer)
    frames_encoded = []
    for rows in frames_channel:
        frames_encoded.append(rows.tobytes(TYPE))
    F=FOUT+str(i)+".wav"
    wf_out = wave.open(F, 'wb')
    wf_out.setnchannels(CHANNELS)
    wf_out.setsampwidth(p_out.get_sample_size(FORMAT))
    wf_out.setframerate(RATE)
    wf_out.writeframes(b''.join(frames_encoded))
    wf_out.close()

p_out.terminate()
