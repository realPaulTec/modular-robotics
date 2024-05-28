from functools import partial
import json
import sys
import numpy as np
import queue
import sounddevice as sd
import pvporcupine
# from openai import OpenAI
import socket
import os
import threading
import signal
import time
import resampy
from pvrecorder import PvRecorder

def exit_handler(signum, frame):
    print('Closing speech...')
    try:
        porcupine.delete()
    except Exception as e:
        pass #print(f"Error closing porcupine: {e}")
    
    try:
        ap.server_socket.close()
        if ap.client_socket:
            ap.client_socket.close()
    except Exception as e:
        pass #print(f"Error closing sockets: {e}")

    os._exit(0)

# Get the full path of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load credentials for OpenAI and Picovoice 
with open('/home/paultec/RTP/src/speech/credentials.json', 'r') as f:
    # Load data with Json lib
    data = json.load(f)
    
    # Extract keys from Json data
    openai_key = data['openai_key']
    picovoice_key = data['picovoice_key']

# Create porcupine wakewrod
porcupine = pvporcupine.create(access_key=picovoice_key, keyword_paths=[
    f"{script_dir}/wakewords/onyx.ppn",
    f"{script_dir}/wakewords/onyx-engage.ppn",
    f"{script_dir}/wakewords/onyx-disengage.ppn",
    f"{script_dir}/wakewords/onyx-forward.ppn",
    f"{script_dir}/wakewords/onyx-reverse.ppn",
    f"{script_dir}/wakewords/onyx-left.ppn",
    f"{script_dir}/wakewords/onyx-right.ppn",
    f"{script_dir}/wakewords/onyx-stop.ppn"
], sensitivities=[
    0.2,
    0.7,
    0.9,
    0.3,
    0.3,
    0.2,
    0.2,
    0.8
])

# Create OpenAI client
# client = OpenAI(api_key=openai_key)

class AudioProcessing:
    # Audio input constants
    SAMPLE_RATE = 44100
    NUM_CHANNELS = 1
    CHANNEL_INDEX = 0

    def __init__(self):
        # Processing variables
        self.input_device = 11
        self.latency = 'low'
        self.chunk_seconds = 2

        # Setup socket communication
        # self.setup_socket()
        
    def setup_socket(self):
        # Setting up socket
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 5000))
        
        # Listening for client
        print('Server listening...')
        self.server_socket.listen()

        # Accept connections
        self.client_socket, addr = self.server_socket.accept()

    def transcribe(self, audio):
        # Get index of wake word
        wake_index = porcupine.process(audio)

        # Print wake inex
        if int(wake_index) != -1    : print(f"W{wake_index}")
        else                        : return

        # Start sending thread
        threading.Thread(target=self.send_data, args=(wake_index,)).start()

    def send_data(self, wake_index):
        # Return if not connected
        if not self.client_socket: return

        # Connecting to client & sending data
        try                     : self.client_socket.sendall(str(wake_index).encode())
        except Exception as e   : print(f"Sending failed: {e}")
    
        # Listen # Engage # Disengage # Forward # Reverse # Left # Right # Stop

    def stream_callback(self, indata, frames, xtime, status, audio_queue, gain_factor=2):
        # Resample data
        audio = resampy.resample(indata[:, 0], self.SAMPLE_RATE, porcupine.sample_rate).astype(np.int16)
          
        audio_queue.put(audio)

    def process_audio(self, audio_queue):
        # Block until the next chunk of audio is available on the queue.
        audio = audio_queue.get()

        # Transcribe the latest audio chunk.
        self.transcribe(audio)

    def record_audio(self):
        # Setup audio queue
        audio_queue = queue.Queue()

        # Setup callback for audio
        callback = partial(self.stream_callback, audio_queue=audio_queue)
        
        # Set block size at given sample rate
        BS = (int(porcupine.frame_length * (self.SAMPLE_RATE/porcupine.sample_rate)) + 1)
        
        # float32 
        # with sd.InputStream(samplerate=porcupine.sample_rate, blocksize=porcupine.frame_length, device=self.input_device, channels=self.NUM_CHANNELS, dtype=np.int16, latency=self.latency, callback=callback):
        with sd.InputStream(samplerate=self.SAMPLE_RATE, blocksize=BS, device=self.input_device, channels=self.NUM_CHANNELS, dtype=np.int16, latency=self.latency, callback=callback):
            while True:
                try:
                    # Process chunks of audio from the queue.
                    self.process_audio(audio_queue)
                except KeyboardInterrupt:
                    print('\n')
                    break

if __name__ == '__main__':
    ap = AudioProcessing()

    # Register the signal handler for SIGINT
    # signal.signal(signal.SIGTERM, exit_handler)
    # signal.signal(signal.SIGINT, exit_handler)

    for index, device in enumerate(PvRecorder.get_audio_devices()):
        print(f"[{index}] {device}")
    
    try:

        
        # print(sd.query_devices(11, 'input'))

        print('Starting...')
        # ap.record_audio()
    finally:
        exit_handler(0, 0)
