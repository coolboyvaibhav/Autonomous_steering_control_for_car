# import RPi.GPIO as GPIO
# import time

# # Define the servo pin
# servo_pin = 11

# # Set up GPIO mode and servo pin
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(servo_pin, GPIO.OUT)

# # Initialize PWM on the servo pin with a 50Hz frequency
# pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
# pwm.start(0)  # Start with the servo off

# def smooth_duty_cycle(target_duty, current_duty, step=0.1, delay=0.02):
#     """
#     Smoothly changes the duty cycle to reach the target value.

#     Args:
#         target_duty (float): The target duty cycle for the servo.
#         current_duty (float): The current duty cycle.
#         step (float): Step size for each increment/decrement to approach the target.
#         delay (float): Delay between steps to control the speed of movement.
#     """
#     while abs(target_duty - current_duty) > step:
#         if current_duty < target_duty:
#             current_duty += step
#         else:
#             current_duty -= step
#         pwm.ChangeDutyCycle(current_duty)
#         time.sleep(delay)
    
#     # Set final target duty cycle and hold
#     pwm.ChangeDutyCycle(target_duty)
#     time.sleep(0.5)  # Allow time for the servo to reach the final position

# # Function to move servo to a specific angle with smooth approach
# def set_servo_angle(angle, current_duty):
#     # Convert angle to target duty cycle
#     target_duty = (angle / 180.0) * 10 + 2.5
#     smooth_duty_cycle(target_duty, current_duty)
#     return target_duty  # Return final duty cycle for future reference

# try:
#     current_duty = 2.5  # Starting position (0 degrees)
#     angle = 5  # Starting angle
#     initialLeft = 5  # Minimum angle
#     initialRight = 45  # Maximum angle

#     while True:
#         current_duty = set_servo_angle(angle, current_duty)
#         print(f"At {angle}°")
        
#         # Increment angle smoothly
#         angle += initialLeft
#         if angle >= initialRight:
#             initialLeft = -5  # Change direction
#         elif angle <= 5:
#             initialLeft = 5  # Reverse direction back

# except KeyboardInterrupt:
#     # Stop PWM and clean up GPIO on exit
#     pwm.stop()
#     GPIO.cleanup()

# Import libraries
import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)  # Pin 11 for servo1, pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)
angleInitial=0

# Function to smoothly set servo to the specified angle
def set_servo_angle(angle):
    duty_cycle = 2 + (angle / 18)  # Map angle 0-180 to duty cycle 2-12
    servo1.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Wait for servo to reach position
    servo1.ChangeDutyCycle(0)  # Turn off pulse to hold position

try:
    while True:
        # Ask user for an angle and turn servo to it continuously
        
        angle = float(input('Enter angle between 0 & 180: '))
        angleInitial=angle
        if True:
            set_servo_angle(angle)

except KeyboardInterrupt:
    # Stop PWM and clean up GPIO on exit
    servo1.stop()
    GPIO.cleanup()
    print("Goodbye!")
