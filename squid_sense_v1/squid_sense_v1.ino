/**
 * Sprint 2 Code
 * Sense
 * SquidBot
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 10/25/2017
 * Version 1
 */
//library for serial communication
#include <EasyTransfer.h>
#include <SoftwareSerial.h> //need this library to run Software Serial

//libraries included to use PixyCam
#include <SPI.h> 
#include <Wire.h> 
//#include <PixyI2C.h>
#include <Pixy.h>

//#include "PixyUART.h"


//library included to use servos
#include<Servo.h>

//libraries included to use motor and motion shield
//#include "NineAxesMotion.h" 



//Constants and Global Variables VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
Pixy pixy; //creates PixyCam object to use
EasyTransfer ETin, ETout; //creates serial structures to transfer data

//flood True if hull flooding
//temp true if electronics overheating
boolean flood, temp, estop = false;


const int COLUMNS = 16;
const int ROWS = 1;
const int FLOODPIN = A3; 
const int MAX_BLOCKS = 7;
const int STOP = A0; // Magnetic sensor pin to determine eStop
const int TEMP = A2;
int filter = 0;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float widths[MAX_BLOCKS];
  int16_t signatures[MAX_BLOCKS];
  float positions[MAX_BLOCKS];
  boolean estop;
};

//give a name to the group of data
SEND_DATA_STRUCTURE txdata;


//SETUP ROBOT CODE (RUN ONCE)SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  Serial.begin(4800);
  
  pixy.init();
  //Arduino.begin(4800);

  ETout.begin(details(txdata), &Serial);

  //Serial.println("SETUP");
  pinMode(STOP, INPUT); 
  pinMode(FLOODPIN, INPUT);

  delay(100);

}

//ROBOT CONTROL LOOP (RUNS UNTIL STOP)LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  int n = pixy.getBlocks();
  for (int i=0; i<MAX_BLOCKS; i++) {
    txdata.signatures[i] = 0;
  }
  for (int i=0; i<min(n,MAX_BLOCKS); i++) {
    txdata.widths[i] = pixy.blocks[i].width;
    txdata.positions[i] = pixy.blocks[i].x;
    txdata.signatures[i] = pixy.blocks[i].signature;
  }
  
  txdata.estop = digitalRead(STOP);

  ETout.sendData();
//  Serial.print(txdata.signatures[0]);
//  Serial.print(txdata.signatures[1]);
//  Serial.print(txdata.signatures[2]);
//  Serial.print(txdata.signatures[3]);
//  Serial.print(txdata.signatures[4]);
//  Serial.print(txdata.signatures[5]);
//  Serial.println(txdata.signatures[6]);
  delay(20);

  //checkFlood();
  //checkTemp();
  
}

//CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC


void checkFlood(){
  int liquidLevel = digitalRead(FLOODPIN);
  if(liquidLevel == HIGH){
    flood = true;
    //Serial.println("FLOOD");
  }
  //else
    //Serial.println("Water Good");
}

void checkTemp(){//temp 150F
  int val=analogRead(TEMP);//Connect LM35 on Analog 0
  float dat = (double) val * (5/10.24); 

  if(dat >= 65.5){
    temp = true;
    //Serial.println("FIRE");
  }
  //else
    //Serial.println("Temp Good");
}








