import struct
import serial
import time
import MovementPriit as MP

robotmotion = MP.RobotMotion()
robotmotion.open()
robotmotion.move(0,0,0)
robotmotion.close()
