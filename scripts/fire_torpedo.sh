#!/usr/bin/env python3
import argparse
import time
import Jetson.GPIO as GPIO

# Torpedo 1 maps to Pin 21
# Torpedo 2 maps to Pin 22
TORPEDO_PINS = {
    1: 21,
    2: 22,
}

# Duration to pulse the solenoid
# TODO: Adjust duration based on testing
PULSE_DURATION = 0.1


def setup_gpio():
  GPIO.setmode(GPIO.BOARD)
  for pin in TORPEDO_PINS.values():
    # Set pins as output and initialize them to LOW
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def fire_torpedo(torpedo_id):
  # Fire both torpedoes
  if torpedo_id == 3:
    pins = list(TORPEDO_PINS.values()) # both pins
    print(f"Firing torpedoes 1 and 2 (pin(s) {pins})...")
  # Fire a specific torpedo
  elif torpedo_id in TORPEDO_PINS:
    pins = [TORPEDO_PINS[torpedo_id]]
    print(f"Firing torpedo {torpedo_id} (pin {pins})...")
  else:
    print(f"Error: Invalid torpedo ID {torpedo_id}. Choose 1, 2, or 3 (both).")
    return

  try:
    # Pulse solenoid to release torpedo
    for pin in pins:
      GPIO.output(pin, GPIO.HIGH)
    time.sleep(PULSE_DURATION)
  finally:
    # Ensure the relay turns off immediately after the pulse
    for pin in pins:
      GPIO.output(pin, GPIO.LOW)
    print(f"Torpedo {torpedo_id} sequence complete.")


def main():
  parser = argparse.ArgumentParser(
      description="Fire Snappy torpedos 1, 2 or 3 (both)."
  )
  parser.add_argument(
      "torpedo",
      type=int,
      choices=[1, 2, 3],
      help="ID of the torpedo to fire (1, 2, 3 (both))",
  )

  args = parser.parse_args()

  try:
    setup_gpio()
    fire_torpedo(args.torpedo)
  finally:
    # Clean up GPIO settings on exit
    GPIO.cleanup()


if __name__ == "__main__":
  main()