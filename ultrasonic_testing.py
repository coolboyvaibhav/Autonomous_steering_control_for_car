import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Send TRIG pulse
GPIO.output(TRIG, False)
time.sleep(2)  # Allow the sensor to settle
GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG, False)

# Monitor ECHO
start_time = time.time()
while time.time() - start_time < 2:  # Wait up to 2 seconds
    if GPIO.input(ECHO):
        print("Signal detected on ECHO!")
        break
else:
    print("No signal detected on ECHO.")

GPIO.cleanup()
