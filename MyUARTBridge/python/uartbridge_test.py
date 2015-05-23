import struct
import serial, time
import threading
import sys

ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.timeout = 0             #non-block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write

stopFlag = 1
def read_from_port(serial):
    alldata = ""
    dlen = 0
    while stopFlag:
        data = serial.read(256)
        if len(data) != 0:
            alldata = alldata + data

        if (dlen > 0) and (len(alldata) > 0):
            print alldata[0]
            alldata = alldata[1:]
            dlen = dlen - 1

        elif(len(alldata) > 2):
            if alldata[0] == '\xAB' and alldata[1] == '\xAD':
                dlen = ord(alldata[2])
                if dlen <= (len(alldata) - 3):
                    print "receive data:" + alldata[3:3+dlen]
                    dlen = 0
                    alldata = alldata[3+dlen:]
                else:
                    dlen -= (len(alldata) - 3)
                    print "receive data:" + alldata[3:]
                    alldata = ""
            elif alldata[0] == '\xAB' and alldata[1] == '\xA5':
                print "send data success len:" + str(ord(alldata[2]))
                #skip 3 bytes
                alldata = alldata[3:]
            else:
                #skip one byte and continue
                alldata = alldata[1:]
                    

def send_str(serial, data):
    length = len(data)
    ser.write(chr(length))
    ser.write(data)
    ser.flushOutput()

try: 
    ser.open()
except Exception, e:
    print "error open serial port: " + str(e)
    exit()

if ser.isOpen():
    try:
        ser.flushInput()
        ser.flushOutput()

        thread = threading.Thread(target=read_from_port, args=(ser,))
        thread.start();

        while(1):
            opt = raw_input("enter string you want to send to remote host, 'quit' to exit: ")
            if opt == 'quit':    
                stopFlag = False
                thread.join()
                sys.exit()
            send_str(ser, opt)
    except Exception, e1:
        print "error communicating...: " + str(e1)

