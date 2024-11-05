import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the ultrasonic sensor
TRIG = 23
ECHO = 24

# Set up the GPIO pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    # Ensure the TRIG pin is low initially
    GPIO.output(TRIG, False)
    time.sleep(0.1)  # Shorter delay to allow the sensor to settle

    # Send a 10us pulse to trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)

    print("TRIG pulse sent")  # Debugging statement

    # Initialize pulse start and end times
    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for the echo to start (pulse goes high)
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    # Wait for the echo to stop (pulse goes low)
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate the duration of the pulse
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s, divide by 2 for round trip
    distance = round(distance, 2)  # Round to 2 decimal places

    # Print debugging information
    print(f"Pulse start: {pulse_start}, Pulse end: {pulse_end}, Duration: {pulse_duration}")
    
    return distance

try:
    while True:
        distance = measure_distance()
        if distance >= 2 and distance <= 400:  # Ultrasonic sensors typically work within 2-400 cm
            print(f"Distance: {distance} cm")
        else:
            print("Out of range or measurement error")
        time.sleep(1)  # Wait 1 second between measurements

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()  # Clean up GPIO on exit
