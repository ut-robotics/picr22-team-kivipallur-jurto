import struct
import serial
import time


def gforward(speed):
        
    disable_failsafe = 0
    speed1 = speed
    speed2 = 0
    speed3 = -speed
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed

def gleft(speed):

    disable_failsafe = 0
    speed1 = speed
    speed2 = speed
    speed3 = speed
    thrower_speed = 00

    andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

    return andmed

def gright(speed):

        disable_failsafe = 0
        speed1 = -speed
        speed2 = -speed
        speed3 = -speed
        thrower_speed = 00

        andmed = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

        return andmed

def gstoprobot():

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
def main():
    ser = serial.Serial('/dev/ttyACM0')  # open serial port
    for i in range(10):
        ser.write(gforward(10))
        time.sleep(0.4)
    for i in range(10):
        ser.write(gright(5))
        time.sleep(0.4)
    for i in range(10):
        ser.write(gleft(5))
        time.sleep(0.4)
        ser.write(gstoprobot())
        time.sleep(0.5)
    ser.close() 

    return 0
    