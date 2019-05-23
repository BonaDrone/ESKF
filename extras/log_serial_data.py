# Quick sketch that stores serial data in a text file
#
# Author: Juan Gallostra
# Date: 16-10-2018

import serial
import time

# Serial parameters
PORT = '/dev/ttyACM0'
BAUDRATE = 115200
LOG_DURATION = 120


def main(port=PORT, baudrate=BAUDRATE, duration=LOG_DURATION):
	# Serial communication object
	serial_com = serial.Serial(port, baudrate)
	intial_time = time.time()

	with open("dataset.csv", "w") as f:
		while time.time() - intial_time < duration:	
        	# read serial data and get the different values received
			raw_data = serial_com.readline().rstrip().split(",")
			# print to terminal so that one can see what's being stored
			print raw_data 
			# Store data in the specified file
			if len(raw_data) == 1 and sum([i.count(".") for i in raw_data]) == 1: #and float(raw_data[2]) > 1.0:
				f.write(",".join(raw_data) + "\n")

	serial_com.close()

if __name__ == "__main__":
	main()
