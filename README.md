# Code useful for home robots
* Motor controls and some sensor integration on the Arduino Nano.
    * Rationale: they have large example sets, are pretty forgiving hardware, cheap, and already are on hand.
* Higher level and more complex logic will be handled through Python on the Raspberry Pi 4.
    * Rationale: adequate guides and similar examples, relatively cheap, and are already on hand.
* This code will be used with differential drive robots, that will share the same root code and much of its functionality, with special configurations being addressed as the time comes.

## To Do:
 * https://learn.adafruit.com/program-an-avr-or-arduino-using-raspberry-pi-gpio-pins/installation
 * Serial exchange from Pi to Arduino with heartbeats and watchdog  https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/
 * Pi Status/Mode indicator LED
https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use?gad_source=1&gclid=CjwKCAiA65m7BhAwEiwAAgu4JD9FtCgHHnpN_hr-w3SNC3MJHgO5yg_ghjP4fyLUVrk08GC1Uknm7RoCm38QAvD_BwE
 * Camera functionality https://www.raspberrypi.com/documentation/computers/camera_software.html#rpicam-apps
 * Establish Modes through controller buttons
    1. Stop
    2. Stadia controlled
    3. OpenCV tracking or Open Bot
 * PicoVoice control of modes
CAN motor control https://forums.raspberrypi.com/viewtopic.php?t=296117
 * ROS integration https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html

## References: 
   * Pi safe shutdown button https://scribles.net/adding-power-switch-on-raspberry-pi/
   * Setup Github on Pi's https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/Fix-GitHubs-support-for-password-authentication-was-removed-error
   * Pi4 case https://www.printables.com/model/106225-modular-snap-together-raspberry-pi-2b3b3b4-case-w-
   * https://www.raspberrypi.com/documentation/computers/
   * Pin out and Pi references https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio
   * https://docs.arduino.cc/learn/electronics/servo-motors/
   * Kitronik Simply Servos Board for Raspberry Pi Pico https://www.sparkfun.com/products/20040
        https://github.com/KitronikLtd/Kitronik-Pico-Simply-Servos-MicroPython

## Build Instructions
 1. Setup hardware
    * Yardbot - endless hours to build
    * Raspberry Pi4 and supplies
    * Wide FoV Pi camera
    * 5x Jaguar Speed controllers or other PWM motor controller
 2. Wiring
    * Pin layout in Arduino firmware
 3. Install Raspberry Pi OS using Raspberry Pi Imager https://www.raspberrypi.com/software/
 4. Configure wifi (2.4 ghz) and bluetooth connections
 5. Mount, power, and connect
 6. From Terminal (Requires a github personal access token, under developer settings)
    * sudo apt-get install arduino
    * git clone https://github.com/rsmith8/robot.git
  7. ...
