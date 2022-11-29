import serial
import time

ser = serial.Serial('COM4', 9600)
time.sleep(2)  
ser.close()
ser.open()
print(ser.name)


while True:
    ser.setRTS(True)   # send a byte
    print("set high")
    time.sleep(0.5)        # wait 0.5 seconds
    ser.setRTS(False)   # send a byte
    print("set low")
    time.sleep(0.5)

ser.close()