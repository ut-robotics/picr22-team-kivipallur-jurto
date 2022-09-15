import struct
import serial
import time

disable_failsafe = 0
speed1 = 20
speed2 = 0
speed3 = -20
thrower_speed = 00

andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
ser = serial.Serial('/dev/ttyACM0')  # open serial port
for i in range(50): #16V 500mA
    ser.write(andmed)     # write a string
    time.sleep(0.4)

ser.close()