import serial
from time import sleep

port = serial.Serial('/dev/ttyUSB_STM32_ACTIONNEURS', 9600)

#print(port.name)
#port.write(b'b')
#sleep(1)
#port.write(b'a')
#sleep(1)
#port.write(b'b')
#sleep(1)
#port.write(b'a')
#sleep(1)
port.write(b'test')
print(port.readline())


port.close()
