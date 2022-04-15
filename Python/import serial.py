import serial
import time

# NOTE: 
# port name may need to change from COM5 to other COM# depending on the computer usb connection
# baud rate depends on the rate being used in the arduino sketch
# the arduino sketch we are working with is "MPU6050_DMP6" example in the MPU6050 subfolder of I2C Arduino documentation

# Opens specified port with specified baudrate. Returns serial.Serial object.
def initialize_arduino(port, baudrate):
    return serial.Serial(port=port, baudrate=baudrate)

# Calculates the initial offsets for a neutral wrist position based on the first 10 seconds of readings. Returns list of offsets.
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

# Reads one line of data from the port. Returns the line of data that was read.
def read(arduino):
   data = arduino.readline()
   return data

# Parses a string in the format "b’yaw pitch roll ‘\t0.90\t-0.04\t0.04\r\n’" to [0.90, -0.04, 0.04]. Returns a list of yaw, pitch, roll angles.
def parse_read(read):
    read = read.split()
    yaw_pitch_roll_strlist = read[3:6]
    return [float(i) for i in yaw_pitch_roll_strlist]

# Calculates the relative yaw, pitch, roll angles between two lists of yaw, pitch, roll angles. Returns a list of relative yaw, pitch, roll angles.
def relative_angles(ypr1, ypr2):
    delta_ypr = []
    for i in range(3):
        delta = ypr1[i] - ypr2[i]
        delta_ypr.append(delta)
    return delta_ypr

# Reads a line 
def record_angles(YPR_list, arduino1, arduino2):
    read1 = read(arduino1)
    read2 = read(arduino2)

    ypr1 = parse_read(read1)
    ypr2 = parse_read(read2)
    orientation = relative_angles(ypr1, ypr2)
    YPR_list[0].append(orientation[0])
    YPR_list[1].append(orientation[1])
    YPR_list[2].append(orientation[2])

# Key function of the program. Initializes serial ports. Calculates offsets, sets constants for risk thresholds and risk period. Contains infinite loop that continuously reads from two serial ports, calculates relative angles, determines if angle thresholds have been exceeded, prints warning to console if appropriate. 
def read_print_loop():
    baudrate = 10
    arduino1 = initialize_arduino('COM5', baudrate)
    arduino2 = initialize_arduino('COM9', baudrate)

    offset_YPR = zero_readings(baudrate, arduino1, arduino2)

    read_buffer = [[], [], []]
    # 5 second risk period for testing purposes
    risk_period = 5
    # Angle thresholds for increased carpal tunnel pressure based on literature reviews
    ulnar_limit = 14.5
    radial_limit = -21.8
    flexion_limit = -48.6
    extension_limit = 32.7

    # Infinite loop
    while True:
        reading_storage_YPR = [[], [], []]

        # Collects baudrate number of readings
        while (len(reading_storage_YPR[0]) < baudrate):
            record_angles(reading_storage_YPR, arduino1, arduino2)

        # Averages baudrate number of readings to get an average reading per second
        Y_per_second = sum(reading_storage_YPR[0]) / baudrate
        P_per_second = sum(reading_storage_YPR[1]) / baudrate
        R_per_second = sum(reading_storage_YPR[2]) / baudrate

        Y_adjusted = round((Y_per_second - offset_YPR[0]), 1)
        P_adjusted = round((P_per_second - offset_YPR[1]), 1)
        R_adjusted = round((R_per_second - offset_YPR[2]), 1)

        # X-axis is Roll, Y-axis, is Pitch, Z-axis is Yaw
        # print("yaw: " + str(Y_per_second) , "pitch: " + str(P_per_second), "roll: " + str(R_per_second))
        formatted_string = f"x-axis: {R_adjusted}, y-axis: {P_adjusted}, z-axis: {Y_adjusted}"

        # Optional line that prints the average wrist angle measurement per second to the console
        # print(formatted_string)

        # Maintains a list with a length equivalent to the risk_period of the most recent average per second readings
        if (len(read_buffer[0]) < risk_period):
            read_buffer[0].append(Y_adjusted)
            read_buffer[1].append(P_adjusted)
            read_buffer[2].append(R_adjusted)
        else:
            read_buffer[0].pop(0)
            read_buffer[1].pop(0)
            read_buffer[2].pop(0)

            read_buffer[0].append(Y_adjusted)
            read_buffer[1].append(P_adjusted)
            read_buffer[2].append(R_adjusted)

            # Conditional blocks that check if a specific wrist angle exceeded its threshold on average over the risk_period and if the wrist angle is currently exceeding the threshold. If both conditions are true, then an appropriate warning is printed to the console.
            if ((sum(read_buffer[0]) / risk_period) > ulnar_limit and read_buffer[0][-1] > ulnar_limit):
                print("Ulnar deviation limit exceeded!")

            if ((sum(read_buffer[0]) / risk_period) < radial_limit and read_buffer[0][-1] < radial_limit):

                print("Radial deviation limit exceeded!")

            if ((sum(read_buffer[2]) / risk_period) < flexion_limit and read_buffer[2][-1] < flexion_limit):
                print("Wrist flexion limit exceeded!")

            if ((sum(read_buffer[2]) / risk_period) > extension_limit and read_buffer[2][-1] > extension_limit):
                print("Wrist extension limit exceeded!")


def main():
    read_print_loop()

if __name__ == "__main__":
    main()