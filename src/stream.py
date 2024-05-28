import socket
import pickle
import struct
import time
import copy

HOST = '192.168.70.62'

PORT = 65432

# Send tracking data to desktop
def send_data(tracking_data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        data = pickle.dumps(tracking_data)
        # Prefix each message with a 4-byte length (network byte order)
        message = struct.pack('>I', len(data)) + data
        
        s.connect((HOST, PORT))
        s.sendall(message)

# Convert defaultdict to a regular dict for pickling
def convert_for_sending(tracking):
    clusters_dict = copy.deepcopy(dict(tracking.clusters))

    # Remove 'points_cartesian' from each cluster
    for cluster in clusters_dict.values():
        del cluster['points_cartesian']

    tracking_data = {
        'tracking'      : tracking.tracking,
        'tracked_point' : tracking.tracked_point,
        'prediction'    : tracking.prediction,
        'clusters'      : clusters_dict,
        'override'      : tracking.override,
        'accuracy'      : tracking.kalman_accuracy,
        'heading'       : tracking.heading
    }

    return tracking_data

def streamer(tracking, stop_control):
    while True:
        # Sync data stream with tracking
        tracking.send_data.wait()

        # Prepare data to send
        tracking_data = convert_for_sending(tracking)

        # Send data to desktop
        try:
            send_data(tracking_data)
        except Exception as e:
            # print(f'\nERROR {e}')
            stop_control.set()

        # Clear send_data event
        tracking.send_data.clear()
        time.sleep(0.1)

def movement_setter(data, forward, reverse, left, right, stop, reset=False):
    # Clear all directions
    forward.clear(); reverse.clear(); left.clear(); right.clear(); stop.clear()

    # Directions list
    directions = [forward, reverse, left, right, stop]

    # Set according to data
    if not reset    : directions[data-3].set()

def receive_speech(terminate_speech, engage, disengage, forward, reverse, left, right, stop):
    # Initial server setup
    client_socket = None
    
    # Attempt to connect until successful or terminated
    try:
        # Setup client socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print('Attempting to connect to server...')
        # Connect to speech server
        client_socket.connect(('localhost', 5000))
        client_socket.settimeout(2)
        print('Connected to server.')

        while not terminate_speech.is_set():
            # Attempt to receive data
            try                         : data = int(client_socket.recv(1024).decode())
            except Exception            : continue

            print(data)

            # State machine
            if      data == -1      : continue
            elif    data == 0       : pass                                                                                                                                      # Listen
            elif    data == 1       : engage.set(); disengage.clear(); movement_setter(data, forward, reverse, left, right, stop, reset=True); print('\rENGAGED', end='')       # Engage
            elif    data == 2       : disengage.set(); engage.clear(); movement_setter(data, forward, reverse, left, right, stop, reset=True); print('\rDISENGAGED', end='')    # Disengage
            else                    : movement_setter(data, forward, reverse, left, right, stop)                                                                                # Forward, Reverse, Left, Right, Stop
    finally:
        # Closing the client
        print(f'Closing client...')
        client_socket.close()

if __name__ == '__main__':
    receive_speech()
