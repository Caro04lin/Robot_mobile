""" Ultrason pour la détection d'obstacle
"""
# sensors/ultrason_sensor.py

import RPi.GPIO as GPIO
import time

TRIG = 11
ECHO = 8

# HC-SR04: wait at least 60 ms between pings.
_MEASURE_INTERVAL_S = 0.065

# Plausible HC-SR04 range. Values outside this range are treated as noise.
MIN_VALID_CM = 2.0
MAX_VALID_CM = 400.0
NO_ECHO_CM = 999.0

# Avoid stopping the robot on one isolated bad low reading such as 140, 999, 8.
REQUIRED_CLOSE_READS = 2
_close_read_count = 0
_last_valid_distance = None


def setup():
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(ECHO, GPIO.IN)
  time.sleep(0.05)


def get_distance():
  # Ensure a clean low level before the trigger pulse.
  GPIO.output(TRIG, GPIO.LOW)
  time.sleep(0.000002)

  # 10 us trigger pulse, per HC-SR04 datasheet.
  GPIO.output(TRIG, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(TRIG, GPIO.LOW)

  # Wait for ECHO rising edge.
  timeout = time.monotonic() + 0.02
  while not GPIO.input(ECHO):
    if time.monotonic() > timeout:
      time.sleep(_MEASURE_INTERVAL_S)
      return NO_ECHO_CM

  t1 = time.monotonic()

  # Wait for ECHO falling edge.
  timeout = time.monotonic() + 0.02
  while GPIO.input(ECHO):
    if time.monotonic() > timeout:
      time.sleep(_MEASURE_INTERVAL_S)
      return NO_ECHO_CM

  t2 = time.monotonic()
  time.sleep(_MEASURE_INTERVAL_S)

  # Speed of sound: 34300 cm/s, round trip so /2.
  return (t2 - t1) * 17150


def _is_valid_distance(distance_cm):
  return MIN_VALID_CM <= distance_cm <= MAX_VALID_CM


def is_obstacle_detected(threshold_cm=30):
  global _close_read_count, _last_valid_distance

  distance = get_distance()

  if not _is_valid_distance(distance):
    _close_read_count = 0
    print(f"Distance ignoree : {distance:.1f} cm")
    return False

  _last_valid_distance = distance

  if distance < threshold_cm:
    _close_read_count += 1
  else:
    _close_read_count = 0

  confirmed = _close_read_count >= REQUIRED_CLOSE_READS
  status = "OBSTACLE" if confirmed else "ok"
  print(
    f"Distance mesuree : {distance:.1f} cm | "
    f"proche={_close_read_count}/{REQUIRED_CLOSE_READS} | {status}"
  )
  return confirmed


def destroy():
  GPIO.cleanup()