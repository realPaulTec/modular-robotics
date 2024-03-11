import socket
import pickle
import struct
import time

HOST = '192.168.53.232'
PORT = 65432

# Send tracking data to desktop
def send_data(tracking_data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        data = pickle.dumps(tracking_data)
        # Prefix each message with a 4-byte length (network byte order)
        message = struct.pack('>I', len(data)) + data
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(message)

# Convert defaultdict to a regular dict for pickling
def convert_for_sending(tracking):
    clusters_dict = dict(tracking.clusters)

    tracking_data = {
        'tracking': tracking.tracking,
        'tracked_point': tracking.tracked_point,
        'prediction': tracking.prediction,
        'clusters': clusters_dict
    }

    return tracking_data

def streamer(tracking, stop_control):
    while True:
        # Prepare data to send
        tracking_data = convert_for_sending(tracking)
        
        # Send data to desktop
        try:
            send_data(tracking_data)
        except Exception as e:
            print(f'\nERROR {e}')
            stop_control.set()

        # Sleep to avoid overwhealming network
        time.sleep(0.2)