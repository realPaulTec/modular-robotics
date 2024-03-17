# import sounddevice as sd
# import numpy as np
# import scipy.io.wavfile as wav
# # import json

# from openai import OpenAI
# client = OpenAI()



# with open('credentials.json') as credentials:
#     client = openai.OpenAI(
#     ) 

# fs = 44100  # Sample rate
# duration = 5  # Duration of recording in seconds

# print("Recording...")
# myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=2, dtype='float64')
# sd.wait()  # Wait until recording is finished
# print("Finished recording.")

# # Save as WAV file
# wav.write('output.wav', fs, np.int16(myrecording * 32767))

# credentials.openai_key
