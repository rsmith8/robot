# Code useful for home robots
Motor controls and some sensor integration on the Arduino Nano. 
 * Rationale: they have large example sets, are pretty forgiving hardware, cheap, and already are on hand.
Higher level and more complex logic will be handled through Python on the Raspberry Pi 4.
  * Rationale: adequate guides and similar examples, relatively cheap, and are already on hand.
This code will be used with differential drive robots, that will share the same root code with special configurations being addressed as the time comes.

## To Do:
 * Pi safe shutdown button
 * Github on Pi's
 * Serial exchange from Pi to Arduino with heartbeats and watchdog
 * Pi Status/Mode indicator LED
 * Camera functionality https://www.raspberrypi.com/documentation/computers/camera_software.html#rpicam-apps
 * Establish Modes through controller buttons
    1. Stop
    2. Stadia controlled
    3. OpenCV tracking or Open Bot
 * PicoVoice control of modes

## References: 
   * https://www.raspberrypi.com/documentation/computers/
   * https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/
   * Pin out and Pi references https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio
   * https://docs.arduino.cc/learn/electronics/servo-motors/
   * Kitronik Simply Servos Board for Raspberry Pi Pico https://www.sparkfun.com/products/20040
        https://github.com/KitronikLtd/Kitronik-Pico-Simply-Servos-MicroPython
   * Safe shudown https://www.youtube.com/watch?v=fyHYSLbhLgU

## Log
 1. Setup hardware
    * Yardbot - endless hours to build
    * Raspberry Pi4 and supplies
    * Wide FoV Pi camera
    * 5x Jaguar Speed controllers or other PWM motor controller
 2. Wiring
    * Pi
        * GPIO reset button
        * Pi Camera
        * Stadia Controller (Bluetooth)
    * Arduino
        1. Left Motor (PWM)
        2. Right Motor (PWM)
        3. Dump Motor (PWM)
        4. ws2811 LED string; Top, Front Left, Front Right
        5. Front Bumper
        6. Rear Bumper
        7. Front Left Sonar
        8. Front Right Sonar
        9. Rear Sonar
        10. ToF Sensor (I2C)
 3. Install Raspberry Pi OS using Raspberry Pi Imager https://www.raspberrypi.com/software/
 4. Configure wifi and bluetooth connections
 5. Mount, power, and connect
 6. ...
