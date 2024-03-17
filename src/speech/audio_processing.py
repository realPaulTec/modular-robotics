from functools import partial
import json
import numpy as np
import queue
import sounddevice as sd
from pydub import AudioSegment
import audioop
import pvporcupine
from openai import OpenAI
import zmq
import os

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
porcupine = pvporcupine.create(access_key=picovoice_key, keyword_paths=[f"{script_dir}/wakewords/onyx-engage.ppn", f"{script_dir}/wakewords/onyx-stop.ppn", f"{script_dir}/wakewords/onyx.ppn"])

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

    def transcribe(self, audio):
        # print(audioop.rms(audio.tobytes(), 2))
        wake_index = porcupine.process(audio)

        if wake_index == 0:
            print('Engage')
        elif wake_index == 1:
            print('Stop')
        elif wake_index == 2:
            print('listening')

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
        # Calculate block size
        block_size = self.chunk_seconds * self.SAMPLE_RATE

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
    ap.record_audio()
    porcupine.delete()
