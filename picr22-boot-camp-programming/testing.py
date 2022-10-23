import struct
import serial
import time
import MainMovement as MP

robotmotion = MP.RobotMotion()
robotmotion.open()
#robotmotion.move(0,0,0)
for i in range(20):
    robotmotion.sendCommand(1000,0,0,0)
robotmotion.close()
