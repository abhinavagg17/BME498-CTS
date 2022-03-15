import serial
import time

# port name may need to change from COM5 to other COM# depending on the computer usb connection
# baud rate depends on the rate being used in the arduino sketch (300 is the lowest possible rate)
# the arduino sketch we are working with is "MPU6050_DMP6" example in the MPU6050 subfolder of I2C Arduino documentation
#

arduino1 = serial.Serial(port='COM9', baudrate=300)
arduino2 = serial.Serial(port='COM5', baudrate=300)    

def read():
    data1 = arduino1.readline()
    data2 = arduino2.readline()
    return (data1, data2) 

while True:
    val = read()
    print(val, flush=True)
