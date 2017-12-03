#include <avr/wdt.h>
#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(10,11); // RX, TX

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  SoftSerial.begin(9600);
  Serial.println("Setup");
}

void loop() {
//  digitalWrite(2, HIGH);//Enable data transmit

  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, (millis()/200)%2);
  if (Serial.available()) {
    char c = Serial.read();
    Serial.println(c);
    if (c=='r') {
      Serial.println("Rebooting in 4 seconds");
      software_Reboot();
    }
  }
  if (SoftSerial.available()) {
    char c = SoftSerial.read();
    SoftSerial.println(c);
  }
}

void software_Reboot()
{
  wdt_enable(WDTO_4S);
  while(1)
  {
  }
}
