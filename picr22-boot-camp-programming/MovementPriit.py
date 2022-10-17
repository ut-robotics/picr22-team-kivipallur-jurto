from logging.handlers import WatchedFileHandler
import turtle
import math
import numpy as np
import time
import tkinter as tk
import struct 
import serial
import serial.tools.list_ports as serials

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

    
class stateMachine:
    def spin(processedData):
        while True:
            #if it finds more than 1 balls (2) to choose from, it goes to the drive function
            if len(processedData.balls) > 1:
                return
            else:
                #otherwise it spins til it finds some
                RobotMotion.move(0,0,0.2)

    def drive(processedData):
        while True:
            if len(processedData.balls) > 0:
                for i in range(len(processedData.balls)):
                    print("distance: {}".format(processedData.balls[i]))
                    xmiddif = (460 - (processedData.balls[i].x)) * 0.001
                    yfor = (300 - (processedData.balls[i].distance)) * 0.002
                    print("x",xmiddif,"y",yfor)
                    RobotMotion.move(0,yfor,xmiddif)

    def orbit(processedData,basketColor):
        if basketColor == 'b' : color = processedData.basket_b
        else: color = processedData.basket_m
        while True:
            #it doesn't work atm, because color is a string
            if len(color) < 0:
                RobotMotion.move(0.5,0,0.5)
            else:
                #centre basket to frame
                pass
    def throw():
        RobotMotion.sendCommand(thrower_speed,0,0.1,0):
