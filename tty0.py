import serial
newFile = open("filename.txt", "wb")
ser = serial.Serial('/dev/ttyACM0', baudrate=1000000,  timeout=1, parity=serial.PARITY_NONE, bytesize=8, stopbits=1)
ser.flushInput()

while True:
    b=ser.read(1000)
    newFile.write(b)

ser.close()
