import serial
import time

ser = serial.Serial('COM4', 115200,  timeout=.1)
# ser.baudrate = 115200
#ser.port = 'COM4'
ser.close()
ser.open()
print(ser.name)

def write_read(x):
    ser.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = ser.readline()
    return data

while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value


# values = bytearray([101, 202, 101])
# ser.write(values)

# total = 0

# while total < len(values):
#      print(ord(ser.read(1)))
#      total=total+1
    
ser.close()