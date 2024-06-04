from drivers.PySabertooth import Sabertooth
import time

saber = Sabertooth("/dev/ttyTHS1", baudrate=9600, address=128, timeout=0.4)
saber.sendCommand(16, 14)

saber.driveBoth(40, 40)

time.sleep(20)

saber.driveBoth(0, 0)

# for i in range(int(200/5)):
#     saber.driveBoth(100 - 5 * i, 100 - 5 * i)
#     time.sleep(0.5)

