from openai import OpenAI
import pyaudio
import wave

# Set the parameters for the recording
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 32000# 44100
CHUNK = 1024
RECORD_SECONDS = 10
INDEX = 11
WAVE_OUTPUT_FILENAME = "output.wav"

# Initialize pyaudio
audio = pyaudio.PyAudio()

# List all available audio devices
print("Available audio devices:")
for index in range(audio.get_device_count()):
    info = audio.get_device_info_by_index(index)
    print(f"{index}: {info['name']}")

# Start recording
stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, input_device_index=INDEX, frames_per_buffer=CHUNK)

print("Recording...")
frames = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Finished recording.")

# Stop and close the stream
stream.stop_stream()
stream.close()
audio.terminate()

# Save the recorded data as a WAV file
with wave.open(WAVE_OUTPUT_FILENAME, 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))

# # Read API key from credentials.json
# with open('credentials.json', 'r') as f:
#     openai_key = json.load(f)['openai_key']

# # Initialize OpenAI REST client 
# client = OpenAI(api_key=openai_key)
