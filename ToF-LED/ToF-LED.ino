 // Techatronic.com  
 #include <Adafruit_NeoPixel.h>  
 #include <Wire.h>
 byte deviceAddress = 0x10;  // The address of the TF-Luna device is 0x10
 #define PIN 2      // input pin Neopixel is attached to  
 #define NUMPIXELS   5 // number of neopixels in strip  
 Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
 int delayval = 10; // timing delay in milliseconds  
 int redColor = 0;  
 int greenColor = 0;  
 int blueColor = 0;  
 unsigned int distance = 0;
 void setup() {  
  // Initialize the NeoPixel library.  
  pixels.begin();  
  Wire.begin();             // The I2C bus communication starts
  Serial.begin(115200);       // Example Set the baud rate of the serial port to 115200

 }  
 void loop() {  
  setColor();

  Wire.beginTransmission(deviceAddress);  // The I2C data transmission starts
  Wire.write(0x00);                       // Send command
  Wire.endTransmission();                 // The I2C data transfer is complete
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)7);     // Read 7 bytes of data 
  if (Wire.available() == 7) {            // 7 bytes of data are available
    byte data[7];
    for (int i = 0; i < 7; i++) {
      data[i] = Wire.read();              // Read data into an array
    }
    distance = (data[1] << 8) | data[0];                   // DistanceValue
    unsigned int signalStrength = (data[3] << 8) | data[2];             // signal strength

/*    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm  ");
    Serial.print("Signal Strength: ");
    Serial.print(signalStrength);
    Serial.print("\n"); */
  }

  for (int i=0; i < NUMPIXELS; i++)   
  {  
   // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255  
   if (i==0){
    if (distance<=15){
      pixels.setPixelColor(i, pixels.Color(0, 200, 0));  //green, red, blue
    }
    else if (distance<=45){
      pixels.setPixelColor(i, pixels.Color(200, 200, 0));  //green, red, blue
    }
    else if (distance > 45){
      pixels.setPixelColor(i, pixels.Color(200, 0, 0));  //green, red, blue
    }
    else{
      pixels.setPixelColor(i, pixels.Color(200, 200, 200));  //green, red, blue
    }
   }
   else if (i==1){
    pixels.setPixelColor(i, pixels.Color(200, 0, 0));  //green, red, blue  
   }
   else if (i==2){
    pixels.setPixelColor(i, pixels.Color(200, 200, 0));  //green, red, blue  yellow 200,200,0
   }
   else if (i==3){
    pixels.setPixelColor(i, pixels.Color(127, 127, 127));  //green, red, blue  white 127,127,127
   }
   else if (i==4){
    pixels.setPixelColor(i, pixels.Color(0, 200, 0));  //green, red, blue  
   }
   else {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    //pixels.setPixelColor(i, pixels.Color(greenColor, redColor, blueColor));
   }
   // This sends the updated pixel color to the hardware.  
   pixels.show();  
   // Delay for a period of time (in milliseconds).  
   delay(delayval);  
  }  
 }  
 // setColor()  
 // picks random values to set for RGB  
 void setColor(){  
  redColor = random(0, 255);  
  greenColor = random(0,255);  
  blueColor = random(0, 255);  
 } 
