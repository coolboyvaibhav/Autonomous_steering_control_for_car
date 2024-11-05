import RPi.GPIO as GPIO
import time

# Setup GPIO mode and pin number
GPIO.setmode(GPIO.BOARD)  # or GPIO.setmode(GPIO.BCM) based on pin numbering
servo_pin = 11            # Change to the pin you are using (e.g., 11 for BOARD or 17 for BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM with 50Hz (common frequency for servos)
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
pwm.start(0)                   # Start PWM with 0 duty cycle (off)

# Function to set servo angle
def set_servo_angle(angle):
    # Map angle to duty cycle between 2.5 and 12.5%
    duty_cycle = 2.5 + (angle / 180.0) * 10.0
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for servo to reach position
    pwm.ChangeDutyCycle(0)  # Turn off the PWM signal (optional)

try:
    while True:
        # Sweep servo from 0Â° to 180Â°
        for angle in range(0, 181, 10):  # Increase angle in steps
            set_servo_angle(angle)
            time.sleep(0.1)  # Short delay for movement

        # Sweep servo from 180Â° back to 0Â°
        for angle in range(180, -1, -10):  # Decrease angle in steps
            set_servo_angle(angle)
            time.sleep(0.1)  # Short delay for movement

except KeyboardInterrupt:
    # Exit gracefully on interrupt
    print("Exiting program")

finally:
    # Cleanup and stop PWM
    pwm.stop()
    GPIO.cleanup()
