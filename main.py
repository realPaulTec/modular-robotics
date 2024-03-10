from collections import defaultdict
from user_interface import UserInterface
import socket
import struct
import pickle

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 65432      # Port to listen on (non-privileged ports are > 1023)

def receive_data(conn):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(conn, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(conn, msglen)

def recvall(conn, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = conn.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def reconstruct_data(received_data):
    clusters = defaultdict(lambda: {
        'points': [],
        'points_cartesian' : [],
        'count': 0,
        'central_position': (0, 0),
        'mahalanobis_distance': 0
    })
    clusters.update(received_data['clusters'])

    received_data['clusters'] = clusters
    return received_data

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print('Server started, waiting for connections...')
        
        # Setup user interface
        UI = UserInterface(2.5, 0.4 * 1.4, 0.2, 0.5)

        while True:
            conn, addr = s.accept()
            print('Connected by', addr)

            with conn:
                data = receive_data(conn)

                if not data:
                    continue
                
                tracking_data = reconstruct_data(pickle.loads(data))

                UI.update_clusters(tracking_data['tracking'], tracking_data['tracked_point'], tracking_data['prediction'], tracking_data['clusters'])
                terminate = UI.graphing()
                
                if terminate:
                    print("Terminate signal received. Closing connection.")
                    break

if __name__ == '__main__':
    main()
