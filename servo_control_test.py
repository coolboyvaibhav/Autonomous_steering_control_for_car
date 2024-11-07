import RPi.GPIO as GPIO
import time

# Define the servo pin
servo_pin = 11

# Set up the GPIO mode and servo pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM on the servo pin with a 50Hz frequency
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
pwm.start(0)  # Start with the servo off

def smooth_duty_cycle(target_duty, current_duty, step=0.1, delay=0.02):
    """
    Smoothly changes the duty cycle to reach the target value.

    Args:
        target_duty (float): The target duty cycle for the servo.
        current_duty (float): The current duty cycle.
        step (float): The step size for each increment/decrement to approach the target.
        delay (float): Delay between steps to control the speed of movement.
    """
    # Increment or decrement duty cycle gradually to target
    while abs(target_duty - current_duty) > step:
        if current_duty < target_duty:
            current_duty += step
        else:
            current_duty -= step
        pwm.ChangeDutyCycle(current_duty)
        time.sleep(delay)
    
    # Set final target duty cycle and hold
    pwm.ChangeDutyCycle(target_duty)
    time.sleep(0.5)  # Allow time for the servo to reach the final position

# Function to move servo to a specific angle with smooth approach
def set_servo_angle(angle, current_duty):
    # Convert angle to target duty cycle
    target_duty = (1 / 180) * angle*7.5 + 2.5
    smooth_duty_cycle(target_duty, current_duty)
    return target_duty  # Return final duty cycle for future reference

try:
    current_duty = 2.5  # Starting position (0 degrees)
    while True:
        # Move to 90 degrees smoothly
        # current_duty = set_servo_angle(45, current_duty)
        # time.sleep(2)
        # print("at 45")

        # Move to 180 degrees smoothly
        current_duty = set_servo_angle(0, current_duty)
        time.sleep(2)
        print("at 0")
        # # Move to 0 degrees smoothly
        # current_duty = set_servo_angle(135, current_duty)
        # time.sleep(2)
        # print("at 135")

except KeyboardInterrupt:
    # Stop PWM and clean up GPIO on exit
    pwm.stop()
    GPIO.cleanup()


