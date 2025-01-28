/*----Code for Arduino-Nano with pin layout------------
 Top Left
  --
  Pin 4 GND                             (Wired)
  PIN D6 Sonar REAR ECHO                (Wired)
  PIN D8 Sonar  FL ECHO                 (Wired)
  PIN D9 Sonar  FL TRIG                 (Wired)
  PIN D10 Neo                           (Wired) 0)RF-Bumper 1)LF-Bumper 2)Top (G,R,B)
  PIN D11 Sonar FR ECHO                 (Wired)
  PIN D12 Sonar FR TRIG                 (Wired)
  --
 USB
  --
  PIN D13 Sonar REAR TRIG               (Wired)
  PIN A0/D14 PWM Left/Throttle          (wired)
  PIN A1/D15 PWM Right/Steer            (wired)
  PIN A2/D16 PWM Dump			              (wired)
  PIN A4 ToF SDA I2C bus at 115200 baud (wired)
  PIN A5 ToF SCL I2C bus at 115200 baud (wired)
  PIN 29 GND                            (Wired)
  PIN 30 +5V (Vin 7-12V move to 27? +5) (wired)
  --
 Top Right

 Missing and in code:
  PIN 2 Neopixel                        empty
  PIN 7 LED left indicator              ?Something on pin
  PIN 8 LED right indicatorn            ?Something on pin
  PIN A7 Voltage divider                empty
  PWM Pins 3, 5, 6, 9, 10, 11
------------------------------------------------*/

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

byte deviceAddress = 0x10;  // The address of the TF-Luna device is 0x10
#define PIN 10              //Neopixel is attached to  
#define NUMPIXELS   3       // number of neopixels in strip  
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 100;         // timing delay in milliseconds pause between pixels 
unsigned int distance = 0;  // counts connected arduino and stadia devices
int counter=0;
int alivecount=0;

const int leftPwmPin = 14;  // PWM output for left joystick is 14 on yardbot - test board is 6
const int rightPwmPin = 15; // PWM output for right joystick is 15 on yardbot - test board is 6
const int dumpPwmPin = 16;  // PWM output for right joystick is 16 on yardbot - test board is 6
Servo leftPWM;   // create servo object to control a servo
Servo rightPWM;  // create servo object to control a servo
Servo dumpPWM;   // create servo object to control a servo

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  Wire.begin();             // The I2C bus communication starts
  Serial.begin(115200);
  leftPWM.attach(leftPwmPin);  // attaches the servo on pin to the servo object
  rightPWM.attach(rightPwmPin);  // attaches the servo on pin to the servo object
  dumpPWM.attach(dumpPwmPin);  // attaches the servo on pin to the servo object
  pixels.setPixelColor(0, pixels.Color(0, 50, 0)); //GRB
  pixels.setPixelColor(1, pixels.Color(0, 50, 0)); //GRB
  pixels.setPixelColor(2, pixels.Color(0, 50, 0)); //GRB
}

void loop() {
  // The first NeoPixel in a strand is #0, second is 1, all the way up
      

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
      pixels.setPixelColor(0, pixels.Color(100, 0, 0)); //GRB
    
    if (input.startsWith("L")) {
      int leftY = input.substring(1).toInt();
      int leftPwmValue = map(leftY, 0, 255, 135, 45); // Map joystick value to PWM range
      leftPWM.write(leftPwmValue);
      analogWrite(leftPwmPin, leftPwmValue);
      pixels.setPixelColor(0, pixels.Color(0, 0, 100)); //GRB
      alivecount=0;
    } 
    else if (input.startsWith("R")) {
      int rightY = input.substring(1).toInt();
      int rightPwmValue = map(rightY, 0, 255, 45, 135); // Map joystick value to PWM range
      rightPWM.write(rightPwmValue);
      pixels.setPixelColor(1, pixels.Color(0, 0, 100)); //GRB
      alivecount=0;
    }
    else if (input.startsWith("D")) {
      int lower_dump = input.substring(1).toInt();
      int DumpPwmValue = map(lower_dump, 0, 255, 90, 15); // Map joystick value to PWM range
      dumpPWM.write(DumpPwmValue);
      pixels.setPixelColor(2, pixels.Color(0, 0, 100)); //GRB
      alivecount=0;
    }
    else if (input.startsWith("U")) {
      int raise_dump = input.substring(1).toInt();
      int RaisePwmValue = map(raise_dump, 0, 255, 90, 165); // Map joystick value to PWM range
      dumpPWM.write(RaisePwmValue);
      pixels.setPixelColor(2, pixels.Color(0, 0, 100)); //GRB
      alivecount=0;
    }


  }
  else {
      pixels.setPixelColor(2, pixels.Color(50, 50, 0)); //GRB
  }
  if (alivecount > 1000){
      pixels.setPixelColor(0, pixels.Color(50, 50, 50)); //GRB
      pixels.setPixelColor(1, pixels.Color(50, 50, 50)); //GRB    
  }

  if (counter > 10){
    pixels.show();   // Send the updated pixel colors to the hardware.
    counter=0;
  }    

  counter++;
  alivecount++;
}
