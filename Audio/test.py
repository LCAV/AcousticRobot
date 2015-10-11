""" Play and Record Test """

import pyaudio
import wave
import sys

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS=2
RATE=44100
RECORD_SECONDS = 5

if len(sys.argv) < 2:
     print("Plays a wave file.\n\nUsage: %s input.wav output.wav" % sys.argv[0])
     sys.exit(-1)

FIN = sys.argv[1]
FOUT = sys.argv[2]

wf = wave.open(FIN,'rb')

p_out = pyaudio.PyAudio()
p_in = pyaudio.PyAudio()

stream_out = p_out.open(format=p_out.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
stream_in = p_in.open(format=FORMAT,
                      channels=CHANNELS,
                      rate=RATE,
                      input=True,
                      frames_per_buffer=CHUNK)
frames = []

# read data
data = wf.readframes(CHUNK)
# play stream_out
while data != '':
    stream_out.write(data)
    data = wf.readframes(CHUNK)
    data_in = stream_in.read(CHUNK)
    frames.append(data_in)

wf.close()

# stop stream_out
stream_out.stop_stream()
stream_out.close()
stream_in.stop_stream()
stream_in.close()

# close PyAudio
p_in.terminate()

# save audio file
wf = wave.open(FOUT, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p_out.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()

p_out.terminate()
