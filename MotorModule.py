import RPi.GPIO as GPIO
from time import sleep
import cv2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Servo class to control the servo motor based on angle
class Servo:
    def __init__(self, servo_pin):
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # Set frequency to 50Hz
        self.pwm.start(0)  # Initialize with 0 duty cycle (off)

    def set_angle(self, angle):
        # Convert angle to duty cycle
        duty_cycle = 2.5 + (angle / 180.0) * 10  # Maps angle 0-180 to duty cycle 2.5-12.5
        self.pwm.ChangeDutyCycle(duty_cycle)
        sleep(0.3)  # Give time for the servo to reach the position

    def stop(self):
        self.pwm.ChangeDutyCycle(0)  # Stop sending signal to the servo


# Motor class remains unchanged but used alongside Servo class
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


# Webcam capture function
cap = cv2.VideoCapture('./Video/track_vdo_1.mp4')

def getImg(display=False, size=[480, 240]):
    _, img = cap.read()
    img = cv2.resize(img, (size[0], size[1]))

    if display:
        cv2.imshow('IMG', img)
    return img


def main():
    motor = Motor(2, 3, 4, 17, 22, 27)  # Motor GPIO setup
    servo = Servo(18)  # Servo connected to GPIO 18

    # Test motor movement
    motor.move(0.5, 0, 2)
    motor.stop(2)

    # Test servo control by rotating to different angles
    angles = [0, 45, 90, 135, 180]
    for angle in angles:
        print(f"Setting servo to {angle} degrees")
        servo.set_angle(angle)
        sleep(1)

    # Return servo to 90 degrees
    servo.set_angle(90)
    servo.stop()


if __name__ == '__main__':
    main()

    while True:
        img = getImg(True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()


# import RPi.GPIO as GPIO
# from time import sleep

# # Define the GPIO pin for the servo
# servo_pin = 18  # Change this to your connected GPIO pin

# # Set up GPIO and PWM
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servo_pin, GPIO.OUT)
# pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency for standard servos
# pwm.start(0)  # Start PWM with 0 duty cycle

# # Function to set servo angle
# def set_servo_angle(angle):
#     duty_cycle = 2.5 + (angle / 180.0) * 10  # Convert angle to duty cycle
#     pwm.ChangeDutyCycle(duty_cycle)
#     sleep(0.5)  # Allow time for servo to reach the position
#     pwm.ChangeDutyCycle(0)  # Stop sending the signal (optional, depends on servo)

# # Example usage
# try:
#     set_servo_angle(0)   # Move to 0 degrees
#     sleep(1)
#     set_servo_angle(90)  # Move to 90 degrees (center)
#     sleep(1)
#     set_servo_angle(180) # Move to 180 degrees
#     sleep(1)
# finally:
#     pwm.stop()
#     GPIO.cleanup()
