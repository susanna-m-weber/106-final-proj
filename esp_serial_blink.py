import serial #for Serial communication
import time   #for delay functions)
ser = serial.Serial('COM4',9600) #Create Serial port object called arduinoSerialData
ser.close()
time.sleep(2) #wait for 2 secounds for the communication to get established
ser.open()
print("hi")
 
while True:
    ser.writelines(b'1')   # send a byte
    print(ser.readline())
    time.sleep(0.5)        # wait 0.5 seconds
    print("high")
    ser.writelines(b'0')   # send a byte
    print(ser.readline())
    time.sleep(0.5)
    print("low")

ser.close()
