// the number of the LED pin
const int ledPin = 4;  // 4 corresponds to GPIO4 Onboard Cam Flash LED

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;


void setup() {
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
}


void loop() {
  ledcWrite(ledChannel, 1);
  delay(1000);
  ledcWrite(ledChannel, 0);
  delay(1000);
}
