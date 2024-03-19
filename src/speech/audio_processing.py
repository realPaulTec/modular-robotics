from functools import partial
import json
import numpy as np
import queue
import sounddevice as sd
from pydub import AudioSegment
import audioop
import pvporcupine
from openai import OpenAI
import socket
import zmq
import os
import sys

# Get the full path of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load credentials for OpenAI and Picovoice 
with open('/home/paultec/coding/RTP3/src/speech/credentials.json', 'r') as f:
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
])

# Create OpenAI client
client = OpenAI(api_key=openai_key)

class AudioProcessing:
    # Audio input constants
    SAMPLE_RATE = 32000
    NUM_CHANNELS = 1
    CHANNEL_INDEX = 0

    def __init__(self):
        # Processing variables
        self.input_device = 11
        self.latency = 'low'
        self.chunk_seconds = 2

        # Setup socket communication
        self.setup_socket()
        
    def setup_socket(self):
        # Setting up socket
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 5000))
        # self.server_socket.settimeout(5)
        
        # Listening for client
        print('Server listening...')
        self.server_socket.listen()
    
    def transcribe(self, audio):
        # Get index of wake word
        wake_index = porcupine.process(audio)

        # Check that the wake index isn't -1
        if wake_index == -1:    return

        # Connecting to client & sending data
        try                     : self.client_socket.sendall(wake_index.encode())
        except Exception as e   : self.client_socket, addr = self.server_socket.accept(); print(f'Connected to: {addr}')

        # Listen # Engage # Disengage # Forward # Reverse # Left # Right # Stop

    def stream_callback(self, indata, frames, time, status, audio_queue):
        # Add this chunk of audio to the queue.
        audio = indata[:, self.CHANNEL_INDEX].copy()
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
        # float32
        with sd.InputStream(samplerate=porcupine.sample_rate, blocksize=porcupine.frame_length, device=self.input_device, channels=self.NUM_CHANNELS, dtype=np.int16, latency=self.latency, callback=callback):
            while True:
                try:
                    # Process chunks of audio from the queue.
                    self.process_audio(audio_queue)
                except KeyboardInterrupt:
                    print('\n')
                    break

if __name__ == '__main__':
    ap = AudioProcessing()

    try:
        print('Starting...')
        ap.record_audio()
    finally:
        print('Closing speech...')
        
        # Cleanup...
        porcupine.delete()
        ap.server_socket.close()
        ap.client_socket.close()
