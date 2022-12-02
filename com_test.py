import serial
import time
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()

values = bytearray([4, 9, 62, 144, 56, 30, 147, 3, 210, 89, 111, 78, 184, 151, 17, 129])

while True:
	time.sleep(5)
	ser.write(values)
	try:
		print(f'Characters in ser buffer = {ser.in_waiting}')
		for i in range(ser.in_waiting):
			print (ord(ser.read()))
	except:
		print("pass")
 
print("done")
 
ser.close()