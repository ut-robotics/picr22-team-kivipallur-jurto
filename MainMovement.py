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
from Color import Color
from xbox360controller import Xbox360Controller



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
    controller = 4
    enemy_basket_spin = 5
    enemy_basket_approach = 6


class StateMachine():
    def __init__(self, motion, frame_width, frame_height,):
        self.motion = motion
        self.frame_centre_x = frame_width/2
        self.frame_height = frame_height
        self.throwdist = 0
        self.throw_wo_ball = False


    def spin(self, processedData,timeoutframes):
        if timeoutframes > 500:
            return States.enemy_basket_spin

        # if it finds more than 1 ball to choose from, it goes to the drive function
        if len(processedData.balls) > 0:
            #print(processedData.balls)
            return States.drive
        else:
            # otherwise it spins until it finds some
            self.motion.move(0, 0, 1, 0)
        return States.spin

    def drive(self, processedData):
        if len(processedData.balls) > 0:
            ball = processedData.balls[-1]
            #print("distance: {}".format(ball))
            xmiddif = (self.frame_centre_x - (ball.x)) * 0.0075              #approcing ball relative to the ball's x and y coordinate
            yfor = (self.frame_height*0.75 - (ball.distance)) * 0.005
            #print("x: ", xmiddif, ", y: ", yfor)
            self.motion.move(0, yfor, xmiddif, 0)
            if ball.distance >= self.frame_height*0.73:
                # if it is close enough to a ball, it should start orbiting around it to find the basket:
                return States.orbit
            return States.drive
        # if it loses the ball for some reason (opponent takes it), it starts spinning to look for a new one:
        return States.spin
        

    def orbit(self, processedData, basketColor):
        
        basket = processedData.basket_b if basketColor == Color.BLUE else processedData.basket_m

        if len(processedData.balls) > 0 : #checks if robot can see ball
            ball = processedData.balls[-1]
            orbitrad = (self.frame_height*0.90 - ball.distance) *0.004
            orbitvar = (self.frame_centre_x - ball.x )*0.02 #variable
            
            orbitdirmultiplier = 1 #used to assign the direction of orbit
           
            pixel_difference = 3
            # if it has a basket in frame and it is centred, it goes to throw the ball:
            if basket.exists and self.frame_centre_x -pixel_difference  < basket.x < self.frame_centre_x + pixel_difference :

                print("could throw")
                return States.throw
            # if it has a ball and a basket in frame, but the basket isn't centred:
            elif basket.exists:
                orbitdirmultiplier = (self.frame_centre_x - basket.x)*0.004
                #print(orbitdirmultiplier)
                orbitdirmultiplier = max(min(orbitdirmultiplier,0.1),-0.1)

                self.motion.move(orbitdirmultiplier, orbitrad, orbitvar, 0)
                # centre basket and ball to frame
            # if it can't find a basket in the frame, it orbits faster:
            elif not basket.exists:

                self.motion.move(0.3, orbitrad, orbitvar, 0)
            return States.orbit
        # if it lost the ball, it goes to look for a new one:
        return States.spin
        


    def throw(self, processedData,basketColor):

        
        basket = processedData.basket_b if basketColor == Color.BLUE else processedData.basket_m
        thrower_speed = round(3*basket.distance) + 380 # Semi accurate thrower speed calculation based on testing
        #thrower_speed = 800
        print(basket.distance, thrower_speed, self.throw_wo_ball)

        if not self.throw_wo_ball:
            if len(processedData.balls) > 0:
                ball = processedData.balls[-1]
                xmiddif = (self.frame_centre_x*0.976 - (basket.x)) * 0.0055  #proportional driving
                
                self.motion.move(0, 0.3, xmiddif, thrower_speed) #thrower speed for revving the motor
                print(ball.distance)
                if ball.distance > 0.83*self.frame_height:        #NEEDS TUNING
                    self.throw_wo_ball = True
                    self.throwdist = basket.distance

                return States.throw
        else:
            if self.throwdist -20 < basket.distance:
                xmiddif = (self.frame_centre_x*0.976 - (basket.x)) * 0.0055  #proportional driving
                print(self.throwdist,self.throw_wo_ball)
                self.motion.move(0, 0.3, xmiddif, thrower_speed)
                return States.throw

        self.throw_wo_ball = False
        return States.spin



    

    def controller(self,controller):
        print("im here")
        throwerspeed = round(controller.trigger_r.value *1900)
        print(throwerspeed)
        yspeed = controller.axis_l._value_y *-0.9
        xspeed = controller.axis_l._value_x *0.5

        rotate = controller.axis_r._value_x *-0.75
        self.motion.move(xspeed,yspeed,rotate,throwerspeed)
        print(xspeed,yspeed,rotate,throwerspeed)

        return States.controller


    def enemy_basket_spin(self, processedData,basketColor):

        if len(processedData.balls) > 0: # if robot sees ball goes to drive
            return States.drive


        basket = processedData.basket_m if basketColor == Color.BLUE else processedData.basket_b #basket color designation

        if basket.exists:   #if no balls are seen and the enemy basket is seen. Robot starts approaching the enemy basket
            return States.enemy_basket_approach

        self.motion.move(0, 0, 1, 0)

        return States.enemy_basket_spin



    def enemy_basket_approach(self, processedData, basketColor):

        if len(processedData.balls) > 0: # if robot sees ball goes to drive
            return States.drive

        basket = processedData.basket_m if basketColor == Color.BLUE else processedData.basket_b #basket color designation

        if basket.exists:
            if basket.distance > 75: # if 
               xmiddif = (self.frame_centre_x -  (basket.x)) * 0.003  #proportional driving based on basket x coordinate
               self.motion.move(0, 0.4, xmiddif, 0)
               return States.enemy_basket_approach

            else:
                return States.spin
        
        return States.enemy_basket_spin

