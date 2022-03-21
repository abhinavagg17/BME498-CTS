import serial
import time

# NOTE: 
# port name may need to change from COM5 to other COM# depending on the computer usb connection
# baud rate depends on the rate being used in the arduino sketch (300 is the lowest possible rate)
# the arduino sketch we are working with is "MPU6050_DMP6" example in the MPU6050 subfolder of I2C Arduino documentation

# This code has been moved to the initialized_arduino function
# baudrate = 300

# arduino1 = serial.Serial(port='COM5', baudrate=baudrate)
# arduino2 = serial.Serial(port='COM9', baudrate=baudrate)

# we aren't writing to the microcontroller so the write portion of this function does  not matter
# i think there are methods we can apply after readline() to further process the data as it gets read into python (things like strip, for example)
# def write_read(arduino, x):
#     arduino.write(bytes(x, 'utf-8'))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return data

def initialize_arduino(port, baudrate):
    return serial.Serial(port=port, baudrate=baudrate)

def read(arduino):
   data = arduino.readline()
   return data

def parse_read(read):
    read = read.split()
    yaw_pitch_roll_strlist = read[3:6]
    return [float(i) for i in yaw_pitch_roll_strlist]

def relative_angles(ypr1, ypr2):
    delta_ypr = []
    for i in range(3):
        delta = ypr1[i] - ypr2[i]
        delta_ypr.append(delta)
    return delta_ypr

def read_print_loop():

    baudrate = 10
    arduino1 = initialize_arduino('COM5', baudrate)
    arduino2 = initialize_arduino('COM9', baudrate)

    while True:
        reading_storage_Y = []
        reading_storage_P = []
        reading_storage_R = []

        while (len(reading_storage_Y) < baudrate):
            read1 = read(arduino1)
            read2 = read(arduino2)

            ypr1 = parse_read(read1)
            ypr2 = parse_read(read2)
            orientation = relative_angles(ypr1, ypr2)
            reading_storage_Y.append(orientation[0])
            reading_storage_P.append(orientation[1])
            reading_storage_R.append(orientation[2])
            # print(orientation)

        Y_per_second = sum(reading_storage_Y) / baudrate
        P_per_second = sum(reading_storage_P) / baudrate
        R_per_second = sum(reading_storage_R) / baudrate

        print(Y_per_second, P_per_second, R_per_second)

def main():
    read_print_loop()

if __name__ == "__main__":
    main()