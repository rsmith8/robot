"""
Python Program for Raspberry Pi to Interface a Google Stadia Controller via Bluetooth and Send Joystick Y-axis Values to an Arduino Nano via USB Serial

Original Prompt:
Write a Python program that when run on a Raspberry Pi will take a Google Stadia remote control connected by Bluetooth and take the joystick Y-motion for each of the left and right sticks and sends them through USB serial connection to an Arduino Nano.
Steps to Run the Program:
1. Install Required Packages:
   $ sudo apt install python3-evdev python3-serial
2. Pair the Stadia controller with your Raspberry Pi using the Bluetooth settings.
3. Plug the Arduino Nano into the Raspberry Pi via USB and note the serial port (e.g., /dev/ttyUSB0).
4. Save the script as 'stadia_to_arduino.py' and run it:
   $ python3 stadia_to_arduino.py
"""
import evdev  #Read/Write events https://python-evdev.readthedocs.io/en/latest/
import serial #Encapsulates access to serial ports https://pyserial.readthedocs.io/en/latest/pyserial.html
import time   #Time related functions https://docs.python.org/3/library/time.html#

SERIAL_PORT = '/dev/ttyUSB0'  # Adjust to match your Arduino Nano's port
BAUD_RATE = 115200 #Match with Arduino serial BAUD rate
devcount = 0 #Counter for number of devices connected, Stadia and Arduino

def find_stadia_controller(): #Find the Stadia controller device.
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if 'Stadia' in device.name:  # Adjust to match the exact name if needed
            return device
    raise RuntimeError("Stadia controller not found. Make sure it is connected via Bluetooth.")

def main(devcount):
    while devcount!=2:
        # Connect to the Stadia controller
        try:
            stadia_controller = find_stadia_controller()
            print(f"Connected to {stadia_controller.name} at {stadia_controller.path}")
            devcount=1
        except RuntimeError as e:
            print(e)
            devcount=0
            #return #exits from a function entirely, optionally passing back a value.

        # Open serial connection to Arduino
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to Arduino on {SERIAL_PORT}")
            if devcount==1:
                devcount=2
            else:
                devcount=1
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return
        time.sleep(1)            
            
    
        # Read joystick events and send data to Arduino
    try:
        for event in stadia_controller.read_loop(): #Find event codes, install "python -m evdev.evtest"
            if event.type == evdev.ecodes.EV_ABS:   # Get joystick Axis values
                if event.code == evdev.ecodes.ABS_Y:       # Left stick Y-axis
                    left_y = event.value
                    ser.write(f"L{left_y}\n".encode())
                elif event.code == evdev.ecodes.ABS_RZ:    # Right stick Y-axis
                    right_y = event.value
                    ser.write(f"R{right_y}\n".encode())
                elif event.code == evdev.ecodes.ABS_BRAKE: # Left Trigger
                    lower_dump = event.value
                    ser.write(f"D{lower_dump}\n".encode())
                elif event.code == evdev.ecodes.ABS_GAS:   # Right Trigger
                    raise_dump = event.value
                    ser.write(f"U{raise_dump}\n".encode())
    except KeyboardInterrupt: #[CTRL]+[C] interrupt
        print("\nExiting program.")
    finally:
        ser.close()

if __name__ == "__main__":
    main(devcount)