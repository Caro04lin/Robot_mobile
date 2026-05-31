
# sensors/ultrason_sensor.py

import RPi.GPIO as GPIO
import time

TRIG = 11
ECHO = 8

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Envoi impulsion
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG, GPIO.LOW)

    # Attente front montant (avec timeout)
    timeout = time.time() + 0.02
    while not GPIO.input(ECHO):
        if time.time() > timeout:
            return 999

    t1 = time.time()

    # Attente front descendant (avec timeout)
    timeout = time.time() + 0.02
    while GPIO.input(ECHO):
        if time.time() > timeout:
            return 999

    t2 = time.time()

    distance = (t2 - t1) * 340 / 2  # en metres
    return distance * 100  # en cm

def is_obstacle_detected(threshold=10):
    distance = get_distance()
    print(f"Distance mesuree : {distance:.2f} cm")
    return distance < threshold

def destroy():
    GPIO.cleanup()
