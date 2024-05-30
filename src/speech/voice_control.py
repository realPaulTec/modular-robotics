import json
import os
import socket
import threading
from pvrecorder import PvRecorder
import pvporcupine

print('Initializing voice control...')

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
    0.3,
    0.7,
    1.0,
    0.4,
    0.4,
    0.4,
    0.4,
    1.0
])


# Exit handler

def exit_handler():
    print('Closing voice control...')

    try     : porcupine.delete()
    except  : pass

    try     : recorder.delete()
    except  : pass

    try     : server_socket.close()
    except  : pass

    try     : client_socket.close()
    except  : pass

    os._exit(0)

# Get microphone 'USB2.0 Microphone Analog Stereo'
devices = PvRecorder.get_available_devices()


def init_recorder():
    try     : recorder.delete()
    except  : pass

    # Get mic index
    try:
        mic = [idx for idx, s in enumerate(devices) if 'USB2.0 Microphone' in s][0]

    except Exception as e:
        print(f'ERROR: {e}')
        return

    # Initialize recorder
    recorder = PvRecorder(frame_length=porcupine.frame_length, device_index=mic)
    recorder.start()

    return recorder

def setup_socket():
    global server_socket, client_socket
    
    # Setting up socket
    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 5000))
    
    # Listening for client
    server_socket.listen()
    print('VOICE: server listening...')

    # Accept connections
    client_socket, addr = server_socket.accept()

    print(f"VOICE: connected to {addr}")

def send_data(wake_index):
    try                     : client_socket.sendall(str(wake_index).encode())
    except Exception as e   : print(f"VOICE: sending failed: {e}")

try:
    recorder = init_recorder()
    setup_socket()

    while True:
        if recorder == None:
            try:
                recorder = init_recorder()
                print("VOICE: recorder reinitialized!")
            except  : pass

        try:
            pcm = recorder.read()
        except Exception as e:
            recorder = None 

            continue
        
        wake_index = int(porcupine.process(pcm))

        if wake_index != -1:
            print(f"VOICE: detected {wake_index}")
            threading.Thread(target=send_data, daemon=True, args=(wake_index,)).start()

except Exception as e : print(e)
finally:
    exit_handler()
