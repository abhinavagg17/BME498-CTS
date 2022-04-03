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

def zero_readings(baudrate, arduino1, arduino2):
    initial_YPR_reads = [[], [], []]
    
    for i in range(0, baudrate):
        record_angles(initial_YPR_reads, arduino1, arduino2)
        print(str(i) + " Initializing...")

    offsets_YPR = []
    offsets_YPR.append(sum(initial_YPR_reads[0]) / baudrate)
    offsets_YPR.append(sum(initial_YPR_reads[1]) / baudrate)
    offsets_YPR.append(sum(initial_YPR_reads[2]) / baudrate)

    return offsets_YPR

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

def record_angles(YPR_list, arduino1, arduino2):
    read1 = read(arduino1)
    read2 = read(arduino2)

    ypr1 = parse_read(read1)
    ypr2 = parse_read(read2)
    orientation = relative_angles(ypr1, ypr2)
    YPR_list[0].append(orientation[0])
    YPR_list[1].append(orientation[1])
    YPR_list[2].append(orientation[2])

def read_print_loop():

    #print("Enter file name to create and write to:")
    #file_name = input()
    #file_name = 'test.txt'
    #open_file = open(file_name, "a")

    baudrate = 10
    arduino1 = initialize_arduino('COM5', baudrate)
    arduino2 = initialize_arduino('COM9', baudrate)

    offset_YPR = zero_readings(baudrate, arduino1, arduino2)

    while True:
        # reading_storage_Y = []
        # reading_storage_P = []
        # reading_storage_R = []
        reading_storage_YPR = [[], [], []]

        while (len(reading_storage_YPR[0]) < baudrate):
            record_angles(reading_storage_YPR, arduino1, arduino2)

        Y_per_second = sum(reading_storage_YPR[0]) / baudrate
        P_per_second = sum(reading_storage_YPR[1]) / baudrate
        R_per_second = sum(reading_storage_YPR[2]) / baudrate

        Y_adjusted = round((Y_per_second - offset_YPR[0]), 1)
        P_adjusted = round((P_per_second - offset_YPR[1]), 1)
        R_adjusted = round((R_per_second - offset_YPR[2]), 1)

        # X-axis is Roll, Y-axis, is Pitch, Z-axis is Yaw
        # print("yaw: " + str(Y_per_second) , "pitch: " + str(P_per_second), "roll: " + str(R_per_second))
        formatted_string = f"x-axis: {R_adjusted}, y-axis: {P_adjusted}, z-axis: {Y_adjusted}"

        print(formatted_string)
        #open_file.write('test')

def main():
    read_print_loop()

if __name__ == "__main__":
    main()