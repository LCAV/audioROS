import serial
import sys
import os
import struct

# The script need Python 3 and pyserial to run: pip3 install pyserial.
# The script need the serial port to open as arg when executing it (e.g. python read_microphones COM56).
# The script reads 10 ms of data for all 4 microphones and save the data in a csv file.
# Robot selector must be in position 1.

def main():
	if len(sys.argv) > 2:
		baud = sys.argv[2]
	else:
		baud = 2000000

	if len(sys.argv) == 1:
		print('Please give the serial port to use')
		return
	else:
		fdesc = serial.Serial(sys.argv[1], baudrate=baud, timeout=0.1)

	# flush input and output buffers
	fdesc.reset_input_buffer()
	fdesc.reset_output_buffer()
	
	# clear the command line
	fdesc.write(b'\r\n')
	fdesc.flush()
	data = fdesc.read(10)
	
	# request mic data
	fdesc.write(b'mic_data\r\n')
	fdesc.flush()
	garbage = fdesc.read(10) # local echo activated somehow??
	header = fdesc.read(4)
	
	if(header[0]==0xAA and header[1]==0x55): # correct sync
		data = fdesc.read(header[2]*256+header[3])
	else:
		print("data not received correctly")
		return
	
	with open('mic.csv','w') as file:
		for i in range(0, len(data)-1, 8):
			# Convert to signed int and write to csv
			file.write(str(struct.unpack("<h", struct.pack("<BB", data[i], data[i+1]))[0]))
			file.write(",")
			file.write(str(struct.unpack("<h", struct.pack("<BB", data[i+2], data[i+3]))[0]))
			file.write(",")
			file.write(str(struct.unpack("<h", struct.pack("<BB", data[i+4], data[i+5]))[0]))
			file.write(",")
			file.write(str(struct.unpack("<h", struct.pack("<BB", data[i+6], data[i+7]))[0]))
			file.write("\n")

if __name__ == '__main__':
	main()
