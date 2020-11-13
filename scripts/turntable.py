#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
turntable.py: Little script to test turntable functionalities.
"""

import serial
import time

# Mac: 
# SerialIn = serial.Serial("/dev/cu.usbmodem14103",115200)

# Ubuntu:
# to find serial port, run python -m serial.tools.list_ports
SerialIn = serial.Serial("/dev/ttyACM1",115200)

#while 1 :
#    inRaw = SerialIn.readline()
#    print(inRaw)
print('o')
SerialIn.write(b"o")
time.sleep(8)
print('k')
SerialIn.write(b"k")
time.sleep(8)
print('done')
