import RPi.GPIO as GPIO
from time import sleep

# Set up GPIO mode and warnings
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Set the PWM pin and frequency
servo_pin = 16
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency for the servo
pwm.start(0)  # Start with a 0 duty cycle

def angle_to_duty_cycle(angle):
    """Convert an angle in degrees to a duty cycle percentage."""
    return 2.5 + (angle / 180.0) * 10  # Maps 0-180° to duty cycle 2.5-12.5

def smooth_transition(target_duty, current_duty, step=0.1, delay=0.02):
    """Gradually adjust the PWM signal to transition smoothly to the target duty cycle."""
    while abs(target_duty - current_duty) > step:
        if current_duty < target_duty:
            current_duty += step
        else:
            current_duty -= step
        pwm.ChangeDutyCycle(current_duty)
        sleep(delay)  # Control the speed of the transition
    
    # Set final target duty cycle and hold
    pwm.ChangeDutyCycle(target_duty)
    sleep(0.5)  # Stabilize at the final position

try:
    current_duty = 2.5  # Initial position (0 degrees)
    while True:
        # Ask the user for the new angle between 0 and 180 degrees
        angle = float(input("Enter an angle between 0 and 180 degrees: "))
        if 0 <= angle <= 180:
            # Calculate the target duty cycle for the desired angle
            target_duty = angle_to_duty_cycle(angle)
            
            # Only move if the new angle differs from the current position
            if target_duty != current_duty:
                smooth_transition(target_duty, current_duty)
                current_duty = target_duty  # Update the current position
                print(f"Servo moved to {angle} degrees.")
            else:
                print("No change in angle, servo remains in the same position.")
        else:
            print("Invalid angle. Please enter a value between 0 and 180.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Stop PWM and clean up GPIO settings
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete.")
