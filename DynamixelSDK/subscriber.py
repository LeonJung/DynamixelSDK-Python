import time
from time import sleep
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=1000000,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS,
)

# port = "\\\\.\\CNCA0"
# ser = serial.Serial(port, 38400, timeout=0)

while True:
    data = ser.read(1)
    if len(data) > 0:
        print("got : ", int(data.encode('hex'), 16))

    # sleep(0.5)
    print 'not blocked'

ser.close()



# import serial
# ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
# line = ser.read(20)   # read a '\n' terminated line
# ser.close()
# print line


















# ser.isOpen()

# print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

# input=1
# while 1 :
#     # # get keyboard input
#     # input = raw_input(">> ")
#     #     # Python 3 users
#     #     # input = input(">> ")
#     # if input == 'exit':
#     #     ser.close()
#     #     exit()
#     # else:
#     #     # send the character to the device
#     #     # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
#     #     ser.write(input + '\r\n')
#     out = ''
#     # let's wait one second before reading output (let's give device time to answer)
#     time.sleep(1)
#     while ser.inWaiting() > 0:
#         out += ser.read(1)

#     if out != '':
#         print ">>" + out