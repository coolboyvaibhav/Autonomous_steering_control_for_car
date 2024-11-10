# motor_module.py
# This module allows the creation of robot objects for 2 or 4-wheeled robots. The motor driver used is the L298n.
# The base package used is the Rpi GPIO.

import RPi.GPIO as GPIO
from time import sleep
from MotorModule import Motor  # Ensure this module exists
from lane_detection import getLaneCurve  # Ensure this module exists
import WebcamModule  # Ensure this module exists
import cv2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor:
    def __init__(self, EnaA, In1A, In2A, EnaB, In1B, In2B):
        self.EnaA = EnaA
        self.In1A = In1A
        self.In2A = In2A
        self.EnaB = EnaB
        self.In1B = In1B
        self.In2B = In2B     
        
        GPIO.setup(self.EnaA, GPIO.OUT)
        GPIO.setup(self.In1A, GPIO.OUT)
        GPIO.setup(self.In2A, GPIO.OUT)
        GPIO.setup(self.EnaB, GPIO.OUT)
        GPIO.setup(self.In1B, GPIO.OUT)
        GPIO.setup(self.In2B, GPIO.OUT)
        
        self.pwmA = GPIO.PWM(self.EnaA, 100)
        self.pwmB = GPIO.PWM(self.EnaB, 100)
        self.pwmA.start(0)
        self.pwmB.start(0)
        self.mySpeed = 0

    def move(self, speed=0.5, turn=0, t=0):
        speed *= 100
        turn *= 70
        leftSpeed = speed - turn
        rightSpeed = speed + turn

        leftSpeed = max(-100, min(100, leftSpeed))
        rightSpeed = max(-100, min(100, rightSpeed))
        
        self.pwmA.ChangeDutyCycle(abs(leftSpeed))
        self.pwmB.ChangeDutyCycle(abs(rightSpeed))
        
        if leftSpeed > 0:
            GPIO.output(self.In1A, GPIO.HIGH)
            GPIO.output(self.In2A, GPIO.LOW)
        else:
            GPIO.output(self.In1A, GPIO.LOW)
            GPIO.output(self.In2A, GPIO.HIGH)
        
        if rightSpeed > 0:
            GPIO.output(self.In1B, GPIO.HIGH)
            GPIO.output(self.In2B, GPIO.LOW)
        else:
            GPIO.output(self.In1B, GPIO.LOW)
            GPIO.output(self.In2B, GPIO.HIGH)
        
        sleep(t)

    def stop(self, t=0):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)
        self.mySpeed = 0
        sleep(t)

def main():
    motor = Motor(2, 3, 4, 17, 22, 27)
    
    motor.move(0.5, 0, 2)
    motor.stop(2)
    motor.move(-0.5, 0, 2)
    motor.stop(2)
    motor.move(0, 0.5, 2)
    motor.stop(2)
    motor.move(0, -0.5, 2)
    motor.stop(2)

# Webcam capture
cap = cv2.VideoCapture(0)

def getImg(display=False, size=[480, 240]):
    _, img = cap.read()
    img = cv2.resize(img, (size[0], size[1]))
                
    if display:
        cv2.imshow('IMG', img)
    return img

if __name__ == '__main__':
    main()
    
    while True:
        img = getImg(True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
