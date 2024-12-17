# Created using ChatGPT 
# Drives an RC car with a Stadia Bluetooth enabled controller
# Make sure you have the evdev library installed (pip install evdev) and the RPi.GPIO library (pip install RPi.GPIO)
#    adjust the GPIO pin number and trigger threshold according to your setup

import time 
import evdev
import RPi.GPIO as GPIO

# Configure GPIO pin for PWM output
PWM_PIN = 18 #pin 12
PWM_PIN2 = 12 #pin 32
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN2, GPIO.OUT)
pwmDrive = GPIO.PWM(PWM_PIN, 50)  # Initialize PWM with a frequency of 100 Hz
pwm2 = GPIO.PWM(PWM_PIN2, 50)  # Initialize PWM with a frequency of 100 Hz
pwmDrive.start(7.5)  # Start PWM with duty cycle of 0
pwm2.start(7.5)  # Start PWM with duty cycle of 0


# Find the Stadia controller
devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
stadia_device = None
for device in devices:
    if "Stadia" in device.name:
        stadia_device = device
        break
    if not stadia_device:
        print("Stadia controller not found")
        time.sleep(1)
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]


print("Stadia controller found:", stadia_device.name)

# Define trigger threshold (adjust as needed)
TRIGGER_THRESHOLD = 2000

# Read input events from the Stadia controller
for event in stadia_device.read_loop():
    if event.type == evdev.ecodes.EV_ABS:
        if event.code == evdev.ecodes.ABS_Z:
            trigger_value = event.value
            # Map trigger value to PWM duty cycle (0-100)
            duty_cycle = (((trigger_value / 255) * 5) + 5)
            pwm2.ChangeDutyCycle(duty_cycle) 
            print("Steer value:", trigger_value, "Duty cycle:", duty_cycle)

        if event.code == evdev.ecodes.ABS_Y:
            trigger_value = event.value
            # Map trigger value to PWM duty cycle (0-100)
            duty_cycle = (((trigger_value / 255) * 5) + 5)
            pwmDrive.ChangeDutyCycle(duty_cycle) 
            print("Drive value:", trigger_value, "Duty cycle:", duty_cycle)

