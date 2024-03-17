import socket
import pickle
import struct
import time

# LAPTOP '192.168.46.62' 
# DESKTOP '192.168.53.232'
HOST = '192.168.139.232'
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

def receive_speech(terminate_speech, engage, disengage, listen):
    # Setting up server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 5000))
    server_socket.listen()

    # Connecting to client
    print("\nServer is waiting for a connection...")
    client_socket, addr = server_socket.accept()
    print(f"Connected to {addr}")

    try:
        while True:
            try                 : data = int(client_socket.recv(1024).decode())
            except Exception    : data = -1

            # State machine
            if      data == 0   : engage.set(); disengage.clear()
            elif    data == 1   : disengage.set(); engage.clear()
            elif    data == 2   : listen.set()

            if terminate_speech.is_set(): break

    finally:
        print('Closing server...')

        client_socket.close()
        server_socket.close()

if __name__ == '__main__':
    receive_speech()