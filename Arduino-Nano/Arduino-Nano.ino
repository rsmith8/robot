const int leftPwmPin = 14;  // PWM output for left joystick
const int rightPwmPin = 15; // PWM output for right joystick

void setup() {
  Serial.begin(9600);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(rightPwmPin, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    
    if (input.startsWith("L")) {
      int leftY = input.substring(1).toInt();
      int leftPwmValue = map(leftY, 0, 255, 0, 255); // Map joystick value to PWM range
      analogWrite(leftPwmPin, leftPwmValue);
      Serial.print("Left Y: ");
      Serial.println(leftY);
    } 
    else if (input.startsWith("R")) {
      int rightY = input.substring(1).toInt();
      int rightPwmValue = map(rightY, 0, 255, 0, 255); // Map joystick value to PWM range
      analogWrite(rightPwmPin, rightPwmValue);
      Serial.print("Right Y: ");
      Serial.println(rightY);
    }
  }
}
