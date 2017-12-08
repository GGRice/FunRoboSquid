/**
 * Sprint 2 Code
 * Sense
 * SquidBot
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 12/8/2017
 * Version 1
 */
//library for serial communication
#include <EasyTransfer.h>
#include <SoftwareSerial.h> //need this library to run Software Serial

//libraries included to use PixyCam
//#include <SPI.h>  
#include <PixyI2C.h>
#include <Wire.h>


//library included to use servos
#include<Servo.h>

//libraries included to use motor and motion shield
//#include "NineAxesMotion.h" 



//Constants and Global Variables VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
PixyI2C pixy; //creates PixyCam object to use
EasyTransfer ETin, ETout; //creates serial structures to transfer data
SoftwareSerial Arduino(12, 13);

//flood True if hull flooding
//temp true if electronics overheating
boolean flood, temp, estop = false;


const int COLUMNS = 16;
const int ROWS = 1;
const int FLOODPIN = 3; 
const int MAX_BLOCKS = 6;
const int STOP = 4; // Magnetic sensor pin to determin eStop


struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float widths[MAX_BLOCKS];
  int signatures[MAX_BLOCKS];
  float positions[MAX_BLOCKS];
  boolean estop;
};

//give a name to the group of data
SEND_DATA_STRUCTURE txdata;


//SETUP ROBOT CODE (RUN ONCE)SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  //Serial.begin(9600);
  
  pixy.init();
  Arduino.begin(4800);

  ETout.begin(details(txdata), &Arduino);

  //Serial.println("SETUP");

}

//ROBOT CONTROL LOOP (RUNS UNTIL STOP)LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  //Serial.println("MAIN LOOP");
  //Serial.println(txdata.test);
  for (int i=0; i<MAX_BLOCKS; i++) {
    txdata.signatures[i] = 0;
  }
  for (int i=0; i<min(pixy.getBlocks(),MAX_BLOCKS); i++) {
    txdata.widths[i] = pixy.blocks[i].width;
    txdata.positions[i] = pixy.blocks[i].width;
    txdata.signatures[i] = pixy.blocks[i].signature;
  }
  
  txdata.estop = digitalRead(STOP);

  ETout.sendData();
  delay(50);

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
  //Serial.println("Water Good");
}

void checkTemp(){//temp 150F
  int val=analogRead(A0);//Connect LM35 on Analog 0
  float dat = (double) val * (5/10.24); 

  if(dat >= 65.5){
    temp = true;
    //Serial.println("FIRE");
  }
  //Serial.println("Temp Good");
}








