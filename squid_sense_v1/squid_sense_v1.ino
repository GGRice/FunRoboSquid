/**
 * Sprint 2 Code Outline
 * Sense
 * SquidBot
 * Outline of code for sprint 2
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 10/25/2017
 * Version 1
 */
//library for serial communication
#include <EasyTransfer.h>

//libraries included to use PixyCam
#include <SPI.h>  
#include <Pixy.h>

//library included to use servos
#include<Servo.h>

//libraries included to use motor and motion shield
//#include "NineAxesMotion.h" 

//library for lcd
#include <LiquidCrystal.h>

//Constants and Global Variables VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
Pixy pixy; //creates PixyCam object to use
EasyTransfer ETin, ETout; //creates serial structures to transfer data

//flood True if hull flooding
//temp true if electronics overheating
boolean flood, temp = false;

//lcd screen setup CHANGE DEPENDING ON LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int COLUMNS = 16;
const int ROWS = 1;
const int FLOODPIN = 1; // ?????
const int MAX_BLOCKS = 6;

//Serial structures 
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  Block blocks[MAX_BLOCKS];
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  Block blocks[MAX_BLOCKS];
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


//SETUP ROBOT CODE (RUN ONCE)SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  Serial.begin(9600);
  pixy.init();

  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  lcd.begin(COLUMNS, ROWS);
}

//ROBOT CONTROL LOOP (RUNS UNTIL STOP)LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  for (int i=0; i<min(pixy.getBlocks(),MAX_BLOCKS); i++) {
    txdata.blocks[i] = pixy.blocks[i];
  }
  memset(txdata.blocks,pixy.getBlocks()*sizeof(Block),MAX_BLOCKS*sizeof(Block));
  ETout.sendData();
  
  printLCDScreen();
}

//CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC


void checkFlood(){
  int liquidLevel = digitalRead(FLOODPIN);
  if(liquidLevel == HIGH)
    flood = true;
}

void checkTemp(){//temp 150F
  int val=analogRead(A0);//Connect LM35 on Analog 0
  float dat = (double) val * (5/10.24); 

  if(dat >= 65.5)
    temp = true;
}

void printLCDScreen(){
  checkFlood();
  checkTemp();
  if(!flood && !temp)
    lcd.print("Happy :)");
  else
    lcd.print("PROBLEM!!");

  lcd.display();
}



