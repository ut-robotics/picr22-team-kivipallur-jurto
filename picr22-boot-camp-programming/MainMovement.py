# from logging.handlers import WatchedFileHandler
import math
import numpy as np
import struct 
import serial
import serial.tools.list_ports as serials
#import scipy.optimize as opt
from enum import Enum

class RobotMotion:
    wheelRadius = 0.035 #needs to be remeasured
    wheelDistanceFromCenter = 0.2 #needs to be remeasured
    gearboxReductionRatio = 18.75
    encoderEdgesPerMotorRevolution = 64
    pidControlFrequency = 100
    thrower_speed = 00
    wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * math.pi * wheelRadius * pidControlFrequency)
    wheelAngles = [math.radians(degree) for degree in [120,0,240]] #converts wheel angles from degrees to radians
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
        self.ser.close()

    def move(self, x_speed, y_speed, robotAngularVelocity):
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)
        wheelSpeeds = [robotSpeed * math.cos(robotDirectionAngle - angle) + self.wheelDistanceFromCenter * robotAngularVelocity for angle in self.wheelAngles]
        wheelSpeedsInMainboardUnits = [round(speed*self.wheelSpeedToMainboardUnits) for speed in wheelSpeeds]
        thrower_speed = 0 #temporary
        print(wheelSpeedsInMainboardUnits)
        self.sendCommand(thrower_speed,*wheelSpeedsInMainboardUnits)
        
    def sendCommand(self,thrower_speed,speed1,speed2,speed3):
        self.ser.write(struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, self.disable_failsafe, 0xAAAA))
        self.ser.read(self.ser.in_waiting) #reads as many bytes as there are to read

    
class states(Enum):
    spin = 0
    drive = 1
    orbit = 2
    throw = 3


class stateMachine:
    def spin(self,processedData):
        while True:
            #if it finds more than 1 balls (2) to choose from, it goes to the drive function
            if len(processedData.balls) > 1:
                return states.drive.value
            else:
                #otherwise it spins until it finds some
                RobotMotion.move(RobotMotion,0,0,0.2)


    def drive(self,processedData):
        while True:
            if len(processedData.balls) > 0:
                ball = processedData.balls[0] 
                print("distance: {}".format(ball))
                xmiddif = (460 - (ball.x)) * 0.001
                yfor = (300 - (ball.distance)) * 0.002
                print("x: ",xmiddif,", y: ",yfor)
                RobotMotion.move(RobotMotion,0,yfor,xmiddif)
                #if it is close enough to a ball, it should start orbiting around it to find the basket:
                if ball.distance <= 270:
                    return states.orbit.value
            #if it loses the ball for some reason (opponent takes it), it starts spinning to look for a new one:
            else: 
                return states.spin.value

    def orbit(self,processedData,basketColor):
        basket = processedData.basket_b if basketColor == 'b' else processedData.basket_m
        frame_centre_x = 848/2
        while True:
            if len(processedData.balls) > 0:
                #if it has a basket in frame and it is centred, it goes to throw the ball:
                if basket.exists and frame_centre_x - 10 < basket.x < frame_centre_x + 10:
                    return states.throw.value
                #if it has a ball and a basket in frame, but the basket isn't centred:
                elif basket.exists:
                    RobotMotion.move(RobotMotion,0.1,0,0.1)
                    #centre basket to frame
                #if it can't find a basket in the frame, it orbits faster:
                elif not basket.exists:
                    RobotMotion.move(RobotMotion,0.3,0,0.3)
            #if it lost the ball, it goes to look for a new one:
            else:
                return states.spin.value

    # def SizeToDistance(basket_size, a, b):
    #     return a/(basket_size-b)
    
    def throw(self,processedData):
        # X = [] #size in px
        # Y = [] #distance in mm
        # x_data_fit = np.linspace(min(X),max(X),100)
        # y_data_fit = self.SizeToDistance()
        # optimized_params, pcov = opt.curve_fit(
        
        # )
        thrower_speed = 500
        RobotMotion.sendCommand(thrower_speed,0,1,0)
