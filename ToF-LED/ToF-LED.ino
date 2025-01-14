 // TFsense from Techatronic.com  
 #include <Adafruit_NeoPixel.h>  
 #include <Wire.h>
 byte deviceAddress = 0x10;  // The address of the TF-Luna device is 0x10
 #define PIN 10              //Neopixel is attached to  
 #define NUMPIXELS   3       // number of neopixels in strip  
 Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
 int delayval = 100; // timing delay in milliseconds  

 unsigned int distance = 0;
 void setup() {  
  // Initialize the NeoPixel library.  
  pixels.begin();  
  Wire.begin();             // The I2C bus communication starts
  Serial.begin(115200);       // Example Set the baud rate of the serial port to 115200

 }  
 void loop() {  
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

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm  ");
    Serial.print("Signal Strength: ");
    Serial.print(signalStrength);
    Serial.print("\n"); 
  
    pixels.clear(); // Set all pixel colors to 'off'

    if (distance<=15){
      pixels.setPixelColor(2, pixels.Color(0, 200, 0));  //green, red, blue
    }
    else if (distance<=45){
      pixels.setPixelColor(2, pixels.Color(200, 200, 0));  //green, red, blue
    }
    else if (distance > 45){
      pixels.setPixelColor(2, pixels.Color(200, 0, 0));  //green, red, blue
    }
    else{
      pixels.setPixelColor(2, pixels.Color(200, 200, 200));  //green, red, blue
    }
    pixels.setPixelColor(0, pixels.Color(200, 200, 200));  //green, red, blue  
    pixels.setPixelColor(1, pixels.Color(200, 200, 200));  //green, red, blue  yellow 200,200,0
    pixels.show();  
    // Delay for a period of time (in milliseconds).  
    delay(delayval);  
  }
  else{
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(1, pixels.Color(0, 200, 0));  //green, red, blue  
    pixels.setPixelColor(2, pixels.Color(0, 200, 0));  //green, red, blue  yellow 200,200,0
    pixels.setPixelColor(2, pixels.Color(0, 200, 0));  //green, red, blue  yellow 200,200,0
    pixels.show();  
    // Delay for a period of time (in milliseconds).  
    delay(delayval);  
  }  
 }