import serial
import serial_datagram
import sys
import os
import msgpack

#this script needs Python 3, pyserial and msgpack to run
#pip3 install pyserial
#pip3 install msgpack-python

def send_ping_cmd(serial):
    #in python, we can simply write down the MessagePack message and it is automatically built
    #here is an order called "ping" which has a string as data
    msg = msgpack.packb({'ping': 'Hello there :-)'},  use_single_float=True)
    dtgrm = serial_datagram.encode(msg)
    serial.write(dtgrm)
    serial.flush()

def send_set_led_cmd(serial, led, action):
    #in python, we can simply write down the MessagePack message and it is automatically built
    #here is an order called "ping" which has a string as data
    #max uint8_t
    led = led & 255
    action = action & 255

    msg = msgpack.packb({'set_led': [led,action]},  use_single_float=True)
    dtgrm = serial_datagram.encode(msg)
    serial.write(dtgrm)
    serial.flush()

def send_set_rgb_led_cmd(serial, led, red, green, blue):
    #in python, we can simply write down the MessagePack message and it is automatically built
    #here is an order called "ping" which has a string as data
    #max uint8_t
    led = led & 255
    red = red & 255
    green = green & 255
    blue = blue & 255

    msg = msgpack.packb({'set_rgb_led': [led, red, green, blue]},  use_single_float=True)
    dtgrm = serial_datagram.encode(msg)
    serial.write(dtgrm)
    serial.flush()

#the script need the serial port to open as arg when executing it
def main():
    if len(sys.argv) > 2:
        baud = sys.argv[2]
    else:
        baud = 115200

    if len(sys.argv) == 1:
        print('Please give the serial port to use')
        return
    else:
        fdesc = serial.Serial(sys.argv[1], baudrate=baud)

    #unsed to align the texts correctly
    names = ('accelerometer','gyroscope', 'time', 'battery', 'distance')
    max_length = len(max(names,key=len))

    #value for the rgb led
    blue = 255

    while True:
        try:
            #return a complete decoded frame
            dtgrm = serial_datagram.read(fdesc)
            #unpack the message pack frame
            data = msgpack.unpackb(dtgrm, use_list=False, encoding='utf-8')

            #{} is replaced by the variables in the .format() function
            #{:<{}} means the text is aligned to the left and the max size of the text is given by 
            #a variable in the .format() function
            #{:6.2f} means the number is a float with 6 digits and a precision of two

            #print acc datas only if they are present in the frame received
            field_to_found = 'acc'
            if(field_to_found in data):
                buf = data[field_to_found]
                print('{:<{}} : x = {:6.2f}, y = {:6.2f}, z = {:6.2f}'.format(names[0], max_length,buf[0], buf[1], buf[2]))
                #send a ping command if we get the acc field in the response
                send_ping_cmd(fdesc) 
                send_set_led_cmd(fdesc, 0, 2)
                if(blue):
                    blue = 0
                else:
                    blue = 255
                send_set_rgb_led_cmd(fdesc, 0, 0, 0, blue)

            #print gyro datas only if they are present in the frame received
            field_to_found = 'gyro'
            if(field_to_found in data):
                buf = data[field_to_found]
                print('{:<{}} : x = {:6.2f}, y = {:6.2f}, z = {:6.2f}'.format(names[1], max_length, buf[0], buf[1], buf[2]))

            #print time data only if they are present in the frame received
            field_to_found = 'time'
            if(field_to_found in data):
                time = data[field_to_found]
                print('{:<{}} : {:.2f} seconds\n'.format(names[2], max_length, time))

            #print time data only if they are present in the frame received
            field_to_found = 'batt'
            if(field_to_found in data):
                batt = data[field_to_found]
                print('{:<{}} : {:.2f} V\n'.format(names[3], max_length, batt))

            #print time data only if they are present in the frame received
            field_to_found = 'dist'
            if(field_to_found in data):
                dist = data[field_to_found]
                print('{:<{}} : {} mm\n'.format(names[4], max_length, dist))

            #print the ping response
            field_to_found = 'ping'
            if(field_to_found in data):
                print(data)

        except (serial_datagram.CRCMismatchError, serial_datagram.FrameError):
            sys.stderr.write("CRC error")


if __name__ == '__main__':
    main()
