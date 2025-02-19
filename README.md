# Code useful for home robots
* Motor controls and digital sensor integration on the Arduino Nano.
    * Rationale: they have large example sets, are pretty forgiving hardware, cheap, and already are on hand.
* Higher level and more complex logic will be handled through Python on the Raspberry Pi 4.
    * Rationale: adequate guides and similar examples, relatively cheap, and are already on hand.
* This code will be used with differential drive robots, that will share the same root code and much of its functionality, with special configurations being addressed as the time comes.

## To Do:
 * fix Bluetooth connection https://raspberrypi.stackexchange.com/questions/122429/raspberry-pi-4-wont-pair-to-bluetooth-devices/123914#123914 or https://forums.raspberrypi.com/viewtopic.php?t=304000
 	* Edit /usr/lib/systemd/system/bluetooth.service
	* Add -d after ExecStart=/usr/libexec/bluetooth/bluetoothd
	* Save.
	* $ systemctl daemon-reload
	* $ systemctl restart bluetooth
 * voice activation PicoVoice control of modes
 * Serial exchange from Pi to Arduino heartbeats and watchdog  https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/
 * Camera functionality tensor flow or https://www.raspberrypi.com/documentation/computers/camera_software.html#rpicam-apps
 * Establish Modes through controller buttons
    1. Stop
    2. Stadia controlled
    3. OpenCV tracking or Open Bot
 * CAN motor control https://forums.raspberrypi.com/viewtopic.php?t=296117
 * ROS integration https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html

## References: 
   * Pi safe shutdown button https://scribles.net/adding-power-switch-on-raspberry-pi/
   * Setup Arduino on Pi (scroll down to ARM) https://magpi.raspberrypi.com/articles/program-arduino-uno-raspberry-pi
   * Setup Github on Pi https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/Fix-GitHubs-support-for-password-authentication-was-removed-error, then fix SSH keys https://mkyong.com/git/github-keep-asking-for-username-password-when-git-push/
     	1. cd robot
	2. git pull
	3. git add --all #or file name eg. README.md
     	4. git commit -am "Notes..."
     	5. git push
   * Pi4 case https://www.printables.com/model/106225-modular-snap-together-raspberry-pi-2b3b3b4-case-w-
   * https://www.raspberrypi.com/documentation/computers/
   * Pin out and Pi references https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio
   * https://docs.arduino.cc/learn/electronics/servo-motors/
   * Kitronik Simply Servos Board for Raspberry Pi Pico https://www.sparkfun.com/products/20040
        https://github.com/KitronikLtd/Kitronik-Pico-Simply-Servos-MicroPython
   * Mount and SD card https://www.wikihow.com/Linux-How-to-Mount-Drive#:~:text=To%20mount%20a%20drive%20on,to%20mount%20and%20unmount%20drives.
      * lsblk -lf
      * sudo mkdir /media/mydrive
      * sudo mount /dev/XXXX /media/mydrive
      * sudo umount /media/myflashdrive
   * Bluetoothctl issues fixed: https://raspberrypi.stackexchange.com/questions/122429/raspberry-pi-4-wont-pair-to-bluetooth-devices
   * Bluetooth use rfcomm https://forums.raspberrypi.com/viewtopic.php?t=320455

   https://forums.raspberrypi.com/viewtopic.php?p=2292242&hilit=Bluetooth#p2292242
raspi-info

## Build Instructions
 1. Setup hardware
    * Yardbot - endless hours to build
    * Raspberry Pi4 and supplies
    * Wide FoV Pi camera
    * 5x Jaguar Speed controllers or other PWM motor controller
    * circuit breaker and fuse bus
 2. Wiring
    * Pin layout in Arduino firmware
 3. Install Raspberry Pi OS using Raspberry Pi Imager https://www.raspberrypi.com/software/
 4. Configure wifi (2.4 ghz) and bluetooth connections
 5. Mount, power, and connect pinout for arduino firmware
 6. From Terminal (Requires a github personal access token, under developer settings)
    * git clone https://github.com/rsmith8/robot.git
    * setup Arduino https://magpi.raspberrypi.com/articles/program-arduino-uno-raspberry-pi
      * install nano firmware
    * sudo apt install python3-evdev
       * python -m evdev.evtest
    * start script Pi.py: sudo nano /etc/xdg/autostart/display.desktop https://www.makeuseof.com/how-to-run-a-raspberry-pi-program-script-at-startup/
        [Desktop Entry]
        Name=robot
        Exec=/usr/bin/python3 /home/rs/robot/Pi.py
    *python3 -m pip config set global.break-system-packages true
    * pip3 install pvorca

 7. ...
