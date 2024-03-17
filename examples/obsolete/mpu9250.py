import smbus
import time
import math
import numpy as np

bus = smbus.SMBus(1)  # Start communication
address = 0x68  # MPU9250 I2C address
mag_address = 0x0C  # Magnetometer I2C address

# Enable bypass mode to access magnetometer
bus.write_byte_data(address, 0x37, 0x02)

# Set magnetometer to continuous measurement mode
bus.write_byte_data(mag_address, 0x0A, 0x16)

time.sleep(0.1)  # Short delay

def read_mag_data():
    # Read magnetometer data registers (0x03 to 0x08)
    data = bus.read_i2c_block_data(mag_address, 0x03, 7)
    # Convert the data
    x = data[0] * 256 + data[1]
    if x > 32767:
        x -= 65536
    y = data[2] * 256 + data[3]
    if y > 32767:
        y -= 65536
    z = data[4] * 256 + data[5]
    if z > 32767:
        z -= 65536
    # Return the data as a tuple
    return (x, y, z)

def calculate_heading(x, y):
    return math.degrees(math.atan2(y, x)) % 360

samples = 720

while True:
    headings = []
        
    # Read magnetometer data
    for i in range(samples):
        x, y, _ = read_mag_data()

        # Calculate heading
        headings.append(calculate_heading(x, y))
    
    heading = np.median(np.array(headings))
    
    print("Heading: %.2f degrees" % heading)

    # Example usage
    mag_data = read_mag_data()
    # print("Magnetometer data: X=%d, Y=%d, Z=%d" % mag_data)

    # time.sleep(0.025)

