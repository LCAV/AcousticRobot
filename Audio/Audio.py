# -*- coding: utf-8 -*-
"""
Audio.py class
==============

Plays a .wav file of your choice on a source and records simultaneously
the response from 1 or more microphones. The source and all microphones must be
connected to a soundcard which is connected to your computer. (Or use your
computer as a soundcard directly)

For each microphone channel, a output .wav-file is created.

The program was tested for MOTU 897HD Soundcard and for the internal speakers
and microphones of a MAC OS X.

"""
from __future__ import print_function, division
import pyaudio
import wave
import sys
import numpy as np

def get_parameters():
    '''
    Gets parameters from command line

    _Parameters_: none

    _Returns_: inputfile, outputfile
    '''
    if len(sys.argv) < 2:
        print("Plays a wave file.\n\nUsage: %s input.wav output.wav" % sys.argv[0])
        sys.exit(-1)
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    output_file = output_file.replace(".wav","")
    return input_file,output_file

class Audio:
    def __init__(self,input_file,output_file,channels=1,index=3,rate=44100,chunk=1024,
                 format_au=pyaudio.paInt16,format_np=np.int16):
        '''
        Constructor

        _Parameters_:
            input_file = path to .wav file to be played (sine sweep or other)
            output_file = path to new file where data results are stored
            (with or without .wav ending)

            [channels] = number of channels to be recorded (1 by default)
            the channels will be recorded in ascending order from 0 to n_channels-1
            all channels will be saved in separate files (output_file0,1,2,...)
            [index] = index of audio device to use (3 by default)
            [rate] = sampling frequency in Hz (44100 by default)
            [chunk] = chunk size (frames per buffer, 1024 by default)
            [format_au] = format of output, in pyaudio.pa... notation (default: pyaudio.paInt16)
            [format_np] = format of output, in numpy.... notation (default: np.int16)
        '''
        self.input_file = input_file
        self.output_file = output_file
        self.channels=channels
        self.index = index
        self.rate=rate
        self.chunk = chunk
        self.format_au = format_au
        self.format_np = format_np
    def play_and_record(self):
        ''' Plays input file and records output channels.
        _Parameters_: none

        _Returns_: frames
        frames is a list with #BUFFER elements
        (#BUFFER ~= Time of input_file * rate / chunk), each element is one frame
        with length chunk*channels.
        the samples for different channels are stored in
        alternating order.
        (for channels x and o, one frame would be [xoxo ... xo])

        if self.channels=1, each element of frames is one long string.

        if self.channels>1, each element of frames is a list of format format_np.
        '''
        wf_in = wave.open(self.input_file,'rb')
        # create managing PyAudio instances
        p_out = pyaudio.PyAudio()
        p_in = pyaudio.PyAudio()
        self.get_device_index(p_in)
        # initialize streams
        stream_out = p_out.open(output_device_index=self.index,
                                format=p_out.get_format_from_width(wf_in.getsampwidth()),
                                channels=wf_in.getnchannels(),
                                rate=wf_in.getframerate(),
                                output=True)
        stream_in = p_in.open(input_device_index=self.index,
                            format=self.format_au,
                            channels=self.channels,
                            rate=self.rate,
                            input=True,
                            frames_per_buffer=self.chunk)
        self.samp_width = p_out.get_sample_size(self.format_au)
        # initialize frames (to store recorded data in)
        frames = []
        frames_decoded = []
        # read data
        data = wf_in.readframes(self.chunk)
        while data != '':
            # play
            stream_out.write(data)
            data = wf_in.readframes(self.chunk)
            # record
            data_in = stream_in.read(self.chunk)
            frames.append(data_in)
            frames_decoded.append(np.fromstring(data_in,self.format_np))
        wf_in.close()

        # stop streams
        stream_out.stop_stream()
        stream_out.close()
        stream_in.stop_stream()
        stream_in.close()

        # close PyAudio
        p_in.terminate()
        p_out.terminate()
        if self.channels > 1:
            return frames_decoded
        else:
            return frames
    def save_wav_files(self,frames):
        '''
        Saves frames in audio files.

        _Parameters_: frames
        frames is a list of #BUFFER elements, each element is one frame.
        (see output of play_and_record for more details)

        _Returns_: 1 if succeeded
        '''
        # save channel data in different audio files
        if self.channels>1:
            # store number of buffers (circa Time * self.rate / self.chunkS)
            frames_matrix=np.matrix(frames)
            BUFFERS = frames_matrix.shape[0]
            # separate channels into two columns
            frames_ordered=frames_matrix.reshape(BUFFERS*self.chunk,self.channels)
            for i in range(self.channels):
                frames_channel = frames_ordered[:,i] # one array of int16 values
                frames_channel = frames_channel.reshape(BUFFERS,self.chunk) #put into original shape
                # recover data format (in one string per buffer)
                frames_encoded = []
                for rows in frames_channel:
                    frames_encoded.append(rows.tobytes())

                F=self.output_file+str(i)+".wav"
                wf_out = wave.open(F, 'wb')
                wf_out.setnchannels(self.channels)
                wf_out.setsampwidth(self.samp_width)
                wf_out.setframerate(self.rate)
                wf_out.writeframes(b''.join(frames_encoded))
                wf_out.close()
        else:
            # save merged data into audio file
            F=self.output_file+".wav"
            wf_out = wave.open(F, 'wb')
            wf_out.setnchannels(self.channels)
            wf_out.setsampwidth(self.samp_width)
            wf_out.setframerate(self.rate)
            wf_out.writeframes(b''.join(frames))
            wf_out.close()
        return 1
    def get_device_index(self,p):
        ''' Updates the audio device id to be used based on user input

        _Parameters_: Audio manager created with pyaudio

        _Returns_: nothing
        '''

        for i in range(p.get_device_count()):
            print("Index ",i,":\n",p.get_device_info_by_index(i))

        self.index = int(input("\nEnter index of Audio device to be used from above list: \n"))

if __name__ == "__main__":
    rate = 44100
    chunk = 1024
    input_file,output_file = get_parameters()
    Au = Audio(input_file,output_file,1,3,rate,chunk)
    frames=Au.play_and_record()
    Au.save_wav_files(frames)
