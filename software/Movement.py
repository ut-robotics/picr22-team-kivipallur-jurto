import struct
import serial
import time

def forward(speed):
    
    disable_failsafe = 0
    speed1 = speed
    speed2 = 0
    speed3 = -speed
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed

def left(speed):

    disable_failsafe = 0
    speed1 = speed
    speed2 = speed
    speed3 = speed
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed

def right(speed):

    disable_failsafe = 0
    speed1 = -speed
    speed2 = -speed
    speed3 = -speed
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed

def stoprobot():

    disable_failsafe = 0
    speed1 = 0
    speed2 = 0
    speed3 = 0
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed


#andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

#for i in range(50): #16V 500mA
#   ser.write(andmed)     # write a string
#    time.sleep(0.4)
#   #print(ser.readlines())

ser = serial.Serial('/dev/ttyACM0')  # open serial port
for i in range(10):
    ser.write(forward(10))
    time.sleep(0.4)
for i in range(10):
    ser.write(right(5))
    time.sleep(0.4)
for i in range(10):
    ser.write(left(5))
    time.sleep(0.4)
    ser.write(stoprobot())
    time.sleep(0.5)
ser.close() 