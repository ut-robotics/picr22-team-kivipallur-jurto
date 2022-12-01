import struct
import serial
import time
import MainMovement as MP

robotmotion = MP.RobotMotion()
#state = MP.StateMachine()
robotmotion.open()
#robotmotion.move(0,0,0)
for i in range(50):
    #state.testmove()
    robotmotion.sendCommand(615,0,0,0)
    time.sleep(0.3)
robotmotion.close()
