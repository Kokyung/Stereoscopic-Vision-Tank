# -*- coding: utf-8 -*-

# Raspberry Pi GPIO Package 
import RPi.GPIO as GPIO
from time import sleep
import time
import sys
import pygame
import os
from adafruit_servokit import ServoKit

os.system("sudo i2cdetect -y 1")

kit = ServoKit(channels=8)

pipe = None

yaw=0 # Vertical.
pitch=1 # Horizontal. 
yawValue = 90 # Yaw percentage value. Never use over 50%.
pitchValue= 90 # Pitch percentage value.

# Reset servo motor position
kit.servo[0].angle = 90
time.sleep(0.1)
kit.servo[1].angle = 90
time.sleep(0.1)

# Motor Status
STOP  = 0
FORWARD  = 1
BACKWARD = 2

# Mortor Channel
CH1 = 0 # Left Wheel
CH2 = 1 # Right Wheel

# PIN Input, Output Setting
OUTPUT = 1
INPUT = 0

# PIN Setting
HIGH = 1
LOW = 0

#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

# 핀 설정 함수
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # Active PWM with 100khz 
    pwm = GPIO.PWM(EN, 100) 
    # Stop PWM first.   
    pwm.start(0) 
    return pwm

# 모터 제어 함수
def setMotorControl(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)  
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    #뒤로
    elif stat == BACKWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

        
# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorControl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorControl(pwmB, IN3, IN4, speed, stat)
        
def MOVE_FORWARD(Lspeed,Rspeed):
    print('forward')
    setMotor(CH1, Rspeed, FORWARD)
    setMotor(CH2, Lspeed, FORWARD)

def MOVE_BACKWARD(Lspeed,Rspeed):
    print('backward')
    setMotor(CH1, Rspeed, BACKWARD)
    setMotor(CH2, Lspeed, BACKWARD)

def MOVE_LEFT_TURN(speed):
    print('left turn')
    setMotor(CH1, speed, FORWARD)
    setMotor(CH2, speed, BACKWARD)

def MOVE_RIGHT_TURN(speed):
    print('right turn')
    setMotor(CH1, speed, BACKWARD)
    setMotor(CH2, speed, FORWARD)

def MOVE_STOP():
    print('stop')
    setMotor(CH1, 0, STOP)
    setMotor(CH2, 0, STOP)


# GPIO 모드 설정 
GPIO.setmode(GPIO.BCM)
      
#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴 
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

axisUpDown = 1                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 0                     # Joystick axis to read for left / right position
axisLeftRightCamera = 3
axisUpDownCamera = 4
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
axisUpDownCameraIverted = False
#buttonTringle = 2
interval = 0.1                          # Time between keyboard updates in seconds, smaller responds faster but uses more processor time


# Setup pygame and key states
global hadEvent
global moveUp
global moveDown
global moveLeft
global moveRight
global moveQuit
global moveLeftRightCamera
global moveUpDownCamera
#global moveDefaultPosition

hadEvent = True
moveUp = False
moveDown = False
moveLeft = False
moveRight = False
moveQuit = False
moveLeftRightCamera = False
moveUpDownCamera = False
#moveDefaultPosition = False

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
screen = pygame.display.set_mode([300,300])
pygame.display.set_caption("VRTank Control - Press [ESC] to quit")

# Function to handle pygame events
def PygameHandler(events):
    # Variables accessible outside this function
    global hadEvent
    global moveUp
    global moveDown
    global moveLeft
    global moveRight
    global moveQuit
    global moveLeftCamera
    global moveRightCamera
    global moveUpCamera
    global moveDownCamera
    #global moveDefaultPosition
    # Handle each event individually
    for event in events:
        if event.type == pygame.QUIT:
            # User exit
            hadEvent = True
            moveQuit = True
        elif event.type == pygame.KEYDOWN:
            # A key has been pressed, see if it is one we want
            hadEvent = True
            if event.key == pygame.K_ESCAPE:
                moveQuit = True
        elif event.type == pygame.KEYUP:
            # A key has been released, see if it is one we want
            hadEvent = True
            if event.key == pygame.K_ESCAPE:
                moveQuit = False
        elif event.type == pygame.JOYAXISMOTION:
            # A joystick has been moved, read axis positions (-1 to +1)
            hadEvent = True
            upDown = joystick.get_axis(axisUpDown)
            leftRight = joystick.get_axis(axisLeftRight)
            LeftRightCamera = joystick.get_axis(axisLeftRightCamera)
            UpDownCamera = joystick.get_axis(axisUpDownCamera)
            #DefaultPosition = joystick.get_button(buttonTringle)
            # Invert any axes which are incorrect
            if axisUpDownInverted:
                upDown = -upDown
            if axisLeftRightInverted:
                leftRight = -leftRight
            if axisUpDownCameraIverted:
                UpDownCamera = - UpDownCamera
            # Determine Up / Down values
            if upDown < -0.1:
                moveUp = True
                moveDown = False
            elif upDown > 0.1:
                moveUp = False
                moveDown = True
            else:
                moveUp = False
                moveDown = False
            # Determine Left / Right values
            if leftRight < -0.7:
                moveLeft = True
                moveRight = False
            elif leftRight > 0.7:
                moveLeft = False
                moveRight = True
            else:
                moveLeft = False
                moveRight = False
            
            if UpDownCamera < -0.5:
                moveUpCamera = True
                moveDownCamera = False
            elif UpDownCamera > 0.5:
                moveUpCamera = False
                moveDownCamera = True
            else:
                moveUpCamera = False
                moveDownCamera = False
                
            if LeftRightCamera < -0.5:
                moveLeftCamera = False
                moveRightCamera = True
            elif LeftRightCamera > 0.5:
                moveLeftCamera = True
                moveRightCamera = False
            else:
                moveLeftCamera = False
                moveRightCamera = False
                
try:
    print('Press [ESC] to quit')
    # Loop indefinitely
    while True:
        # Get the currently pressed keys on the keyboard
        PygameHandler(pygame.event.get())
        if hadEvent:
            # Keys have changed, generate the command list based on keys
            hadEvent = False
            if moveQuit:
                break
            elif moveLeft:   
                MOVE_LEFT_TURN(80)
            elif moveRight:
                MOVE_RIGHT_TURN(80)
            elif moveUp:
                MOVE_FORWARD(80,80)
            elif moveDown:
                MOVE_BACKWARD(80,80)
            elif moveUpCamera:
                yawValue-=10
                if yawValue<0:
                    yawValue=0
                    continue
                else:
                    kit.servo[yaw].angle = yawValue
            elif moveDownCamera:
                yawValue+=10
                if yawValue>90:
                    yawValue=90
                    continue
                else:
                    kit.servo[yaw].angle = yawValue
            elif moveLeftCamera:
                pitchValue-=10
                if pitchValue<0:
                    pitchValue=0
                    continue
                else:
                    kit.servo[pitch].angle = pitchValue
            elif moveRightCamera:
                pitchValue+=10
                if pitchValue>180:
                    pitchValue=180
                    continue
                else:
                    kit.servo[pitch].angle = pitchValue
            else:
                MOVE_STOP()
        # Wait for the interval period
        time.sleep(interval)
    # Disable all drives
    MOVE_STOP()
except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    MOVE_STOP()
    #os.system('sudo killall servod')