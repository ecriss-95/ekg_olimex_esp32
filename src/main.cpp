#include <Arduino.h>
const int analogInPin = 2; // Output Olimex is connected to GPIO2

void setup() {
  Serial.begin(115200);
}

void loop() {
  int sensorValue = analogRead(analogInPin);
  Serial.println(sensorValue);
  delay(10);
}
