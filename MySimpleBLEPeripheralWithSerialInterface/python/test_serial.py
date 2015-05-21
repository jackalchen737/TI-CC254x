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
    while stopFlag:
        data = serial.readline()
        if len(data) != 0:
            print("receive: " + data)

#ID,OPCODE,LEN,DATA,0x00
def start_adv(serial):
    ser.write("\x77\x01\x00\x00")
    ser.flushOutput()

def stop_adv(serial):
    ser.write("\x77\x02\x00\x00")
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
            opt = raw_input("choose cmd >> (0) start adv (1) stop adv (2) exit : ")
            if opt == "0":
                start_adv(ser)
            elif opt == "1":
                stop_adv(ser)
            elif opt == "2":
                stopFlag = False
                thread.join()
                sys.exit()
    except Exception, e1:
        print "error communicating...: " + str(e1)

