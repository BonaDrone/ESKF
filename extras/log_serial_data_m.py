# Quick sketch that stores serial data in a csv file
#
# Author: Juan Gallostra
# Date: 16-10-2018

import serial
import time
from os import listdir
from os.path import isfile, join

# data files' related info
DATA_PATH = "../data/"
DATA_FILENAME = "raw_data_"
DATA_EXTENSION = ".csv"

# Serial parameters
PORT = '/dev/ttyACM0'
BAUDRATE = 115200
LOG_DURATION = 10000

def get_data_file_name(data_path=DATA_PATH, data_filename=DATA_FILENAME, data_extension=DATA_EXTENSION):
	"""
	"""
	# get data files
	datafiles = [f for f in listdir(data_path) if (isfile(join(data_path, f)) and data_filename in f)]
	datafiles += [f for f in listdir(".") if (isfile(join(".", f)) and data_filename in f)]
	# strip names and extensions to get only numbers
	str_numbers = [f.strip(data_filename).strip(data_extension) for f in datafiles]
	# filter empty strings and cast to integers
	numbers = sorted([int(f) for f in str_numbers if f])
	# get next number
	return data_filename + str(numbers[-1]+1) + data_extension

def main(port=PORT, baudrate=BAUDRATE, duration=LOG_DURATION):
	"""
	"""
	# Serial communication object
	serial_com = serial.Serial(port, baudrate)
	intial_time = time.time()

	with open(get_data_file_name(), "w") as f:
		while time.time() - intial_time < duration:
        	# read serial data and get the different values received
			raw_data = serial_com.readline().rstrip().split(",")
			# print to terminal so that one can see what's being stored
			print raw_data 
			# Store data in the specified file
			if len(raw_data) == 13 and sum([i.count(".") for i in raw_data]) == 13:
				f.write(",".join(raw_data) + "\n")

	serial_com.close()

if __name__ == "__main__":
	main()
