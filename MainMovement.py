# from logging.handlers import WatchedFileHandler
import math
from urllib import robotparser
import numpy as np
import time
import struct
import serial
import websocket
import serial.tools.list_ports as serials
from enum import Enum
from json import loads


class RobotMotion:
    wheelRadius = 0.035  
    wheelDistanceFromCenter = 0.2 
    gearboxReductionRatio = 18.75
    encoderEdgesPerMotorRevolution = 64
    pidControlFrequency = 100
    thrower_speed = 00
    wheelSpeedToMainboardUnits = gearboxReductionRatio * \
        encoderEdgesPerMotorRevolution / \
        (2 * math.pi * wheelRadius * pidControlFrequency)
    # converts wheel angles from degrees to radians
    wheelAngles = [math.radians(degree) for degree in [120, 0, 240]]
    disable_failsafe = 0

    def __init__(self):
        self.ser = None

    def open(self):
        for port in serials.comports():
            if port.vid == 1155 and port.pid == 22336:
                self.ser = serial.Serial(port.device)  # open serial port
                print("leidis seriali")
                break
        if self.ser == None:
            raise Exception("Could not establish serial connection")

    def close(self):
        print("sulgesin seriali")
        self.ser.close()

    def sendCommand(self, thrower_speed, speed1, speed2, speed3):
        self.ser.write(struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, self.disable_failsafe, 0xAAAA))
        # reads as many bytes as there are to read
        self.ser.read(self.ser.in_waiting)

    def move(self, x_speed, y_speed, robotAngularVelocity, thrower_speed):
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)
        wheelSpeeds = [robotSpeed * math.cos(robotDirectionAngle - angle) +
                       self.wheelDistanceFromCenter * robotAngularVelocity for angle in self.wheelAngles]
        wheelSpeedsInMainboardUnits = [round(speed*self.wheelSpeedToMainboardUnits) for speed in wheelSpeeds]
        # thrower_speed = 0 #temporary
        # print(wheelSpeedsInMainboardUnits)
        self.sendCommand(thrower_speed, wheelSpeedsInMainboardUnits[0], wheelSpeedsInMainboardUnits[1], wheelSpeedsInMainboardUnits[2])


class States(Enum):
    spin = 0
    drive = 1
    orbit = 2
    throw = 3
    read_ref = 4


class StateMachine():
    def __init__(self, motion):
        self.motion = motion


    def spin(self, processedData):
        # if it finds more than 1 ball to choose from, it goes to the drive function
        if len(processedData.balls) > 0:
            #print(processedData.balls)
            return States.drive
        else:
            # otherwise it spins until it finds some
            self.motion.move(0, 0, 1, 0)
        return States.spin

    def drive(self, processedData):
        y_keskmine = list()
        if len(processedData.balls) > 0:
            ball = processedData.balls[-1]
            #print("distance: {}".format(ball))
            xmiddif = (424 - (ball.x)) * 0.002              #approcing ball relative to the ball's x and y coordinate
            yfor = (360 - (ball.distance)) * 0.005
            #print("x: ", xmiddif, ", y: ", yfor)
            self.motion.move(0, yfor, xmiddif, 0)
            if ball.distance >= 350:
                # if it is close enough to a ball, it should start orbiting around it to find the basket:
                return States.orbit
        # if it loses the ball for some reason (opponent takes it), it starts spinning to look for a new one:
        else:
            return States.spin
        return States.drive

    def orbit(self, processedData, basketColor):
        
        basket = processedData.basket_b if basketColor == 'b' else processedData.basket_m
        
        frame_centre_x = 848/2
       

        if len(processedData.balls) > 0 : #checks if robot can see ball
            ball = processedData.balls[-1]
            orbitrad = (400 - ball.distance) *0.003
            orbitvar = (frame_centre_x - ball.x )*0.009 #variable
            
            orbitdirmultiplier = 1 #used to assign the direction of orbit
           
            pixel_difference = 2
            # if it has a basket in frame and it is centred, it goes to throw the ball:
            if basket.exists and frame_centre_x -pixel_difference  < basket.x < frame_centre_x + pixel_difference:

                print("could throw")
                return States.throw
            # if it has a ball and a basket in frame, but the basket isn't centred:
            elif basket.exists:
                orbitdirmultiplier = (frame_centre_x - basket.x)*0.002
                #print(orbitdirmultiplier)
                if orbitdirmultiplier > 0.1:
                    orbitdirmultiplier = 0.1
                elif orbitdirmultiplier < -0.1:
                    orbitdirmultiplier = -0.1


                self.motion.move(orbitdirmultiplier, orbitrad, orbitvar, 0)
                # centre basket and ball to frame
            # if it can't find a basket in the frame, it orbits faster:
            elif not basket.exists:

                self.motion.move(0.3, orbitrad, orbitvar, 0)
        # if it lost the ball, it goes to look for a new one:
        else:
            return States.spin
        return States.orbit


    def throw(self, processedData,basketColor,basketdistcm):

        basket = processedData.basket_b if basketColor == 'b' else processedData.basket_m
        thrower_speed = 3*basketdistcm + 390 # Semi accurate thrower speed calculation based on testing
        #thrower_speed = 800
        print(basketdistcm, thrower_speed)

        frame_centre_x = (848/2)
        if len(processedData.balls) > 0:
            ball = processedData.balls[-1]
            xmiddif = (frame_centre_x -  (basket.x)) * 0.003  #proportional driving
            
            self.motion.move(0, 0.2, xmiddif, thrower_speed) #thrower speed for revving the motor
            
            if ball.distance >= 280:
                return States.throw
                
        #throwing measurement:
        for n in range(5):
            self.motion.move(0, 0.2, 0, thrower_speed-25) # hardcoded throw
            time.sleep(0.3)
        return States.spin
