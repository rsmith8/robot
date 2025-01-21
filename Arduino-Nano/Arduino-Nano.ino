// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        10 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 3 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 1000 // Time (in milliseconds) to pause between pixels

const int leftPwmPin = 14;  // PWM output for left joystick
const int rightPwmPin = 15; // PWM output for right joystick

int counter=0;

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  Serial.begin(9600);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(rightPwmPin, OUTPUT);
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
      int leftPwmValue = map(leftY, 0, 255, 0, 255); // Map joystick value to PWM range
      analogWrite(leftPwmPin, leftPwmValue);
      pixels.setPixelColor(1, pixels.Color(0, 100, 0)); //GRB
    } 
    else if (input.startsWith("R")) {
      int rightY = input.substring(1).toInt();
      int rightPwmValue = map(rightY, 0, 255, 0, 255); // Map joystick value to PWM range
      analogWrite(rightPwmPin, rightPwmValue);
      pixels.setPixelColor(2, pixels.Color(0, 0, 100)); //GRB
    }

  }
  else {
      pixels.setPixelColor(0, pixels.Color(50, 50, 0)); //GRB
  }
  
  if (counter > 100){
    pixels.show();   // Send the updated pixel colors to the hardware.
    counter=0;
  }

  counter++;
  
}
