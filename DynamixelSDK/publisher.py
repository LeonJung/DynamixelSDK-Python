import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=57600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

arr = [0xff, 0xff, 0xfd, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4e]

for i in arr:
    a = i
    print a

ser.write(arr)


ser.close()


# input=1
# while 1 :
#     # get keyboard input
#     input = raw_input(">> ")
#         # Python 3 users
#         # input = input(">> ")
#     if input == 'exit':
#         ser.close()
#         exit()
#     else:
#         # send the character to the device
#         # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
#         # ser.write(input + '\r\n')
#         ser.write(input)

#         out = ''
#         # let's wait one second before reading output (let's give device time to answer)
#         time.sleep(1)
#         while ser.inWaiting() > 0:
#             out += ser.read(1)

#         if out != '':
#             print ">>" + out