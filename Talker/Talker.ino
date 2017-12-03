#include <avr/wdt.h>
#include <SoftwareSerial.h>

//SoftwareSerial ArduinoLink(10, 11); // RX, TX

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);
//  ArduinoLink.begin(9600);
  Serial.println("Setup");
}

void loop() {
  Serial.println("Hi");
//  ArduinoLink.println("Hi");
}
