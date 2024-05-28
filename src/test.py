import serial
import time

ser = serial.Serial('/dev/ttyTHS1', 9600)  # Adjust the port and baud rate if needed

address = 128

def lerp(x, out_min, out_max, in_min=-100, in_max=100):
    # clamp values for x
    x = max(min(x, in_max), in_min)

    # Return lerped values
    return int(round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

# -100 - 100
def motorA(speed):
    speed = lerp(speed, 1, 127)
    ser.write(bytes([speed]))

# -100 - 100
def motorB(speed):
    speed = lerp(speed, 128, 255)
    ser.write(bytes([speed]))

motorA(0)

time.sleep(1)

motorB(0)

time.sleep(2)

ser.close()
