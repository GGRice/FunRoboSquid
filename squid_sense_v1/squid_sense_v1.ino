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
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NineAxesMotion.h" 


//Constants and Global Variables VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
Pixy pixy; //creates PixyCam object to use
EasyTransfer ETin, ETout; //creates serial structures to transfer data

//flood True if hull flooding
//temp true if electronics overheating
boolean flood, temp = False;

//lcd screen setup CHANGE DEPENDING ON LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//array to hold info about every bouy
//c=colour, x=x pos, y=y pos, s=size
//1=blue, 2=green, 3=red, 4=home
//array = [(int c1=0, int x1=0, int y1=0, int s1=0),
//          (int c2=0, int x2=0, int y2=0,int s2=0),
//          (int c3=0, int x3=0, int y3=0, int s3=0),
//          (int c4=0, int x4=0, int y4=0, int s4=0)];


//Serial structures 
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  array blocks;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  array blocks;
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
  blocks = pixy.getBlocks();
  
  //just sending, not recieving?
  txdata.blocks = blocks;
  
  Etout.sendData();
  
  printLCDScreen();
}

//CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC


void flood(){
  liquidLevel = digitalRead(FLOODPIN);
  if(liquidLevel == HIGH)
    flood = true;
}

void temp(){//temp 150F
  val=analogRead(A0);//Connect LM35 on Analog 0
  dat = (double) val * (5/10.24); 

  if(dat >= 65.5)
    temp = true;
}

void printLCDScreen(){
  flood();
  temp();
  if(!flood && !temp)
    lcd.print("Happy :)")
  else
    lcd.print("PROBLEM!!")

  lcd.display();
}

