/**
 * Sprint 2 Code Outline
 * Think/Act
 * SquidBot
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 11/7/2017
 * Version 1
 */

//library for Serial Transfer
#include <EasyTransfer.h>

//libraries included to use PixyCam
#include <SPI.h>  
#include <Pixy.h>

//library included to use servos
#include<Servo.h>

//libraries included to use motion shield
#include "NineAxesMotion.h" 
#include <Wire.h>


// CONSTANTS AND GLOBAL VARIABLES VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// Constants
enum {RIGHT=-1, NONE=0, LEFT=1, STRAIGHT=2}; // Directions
enum {RED=3, YELLOW=4, WHITE=5, HOME=6, DANCE=7, LOOP=8}; // Targets
const int APPROACH_DIST = 10; // Distance from target to start turning, in inches
const float K_P = 1.0; // Proportional constant for feedback control
const int FORWARD_VELOCITY = 150; // Pump output for normal swimming
const int TURNING_VELOCITY = 150; // Pump output for turning
const int MAX_MISSION_LENGTH = 10; // Maximum number of targets in a mission
const int CAMERA_RATIO = 1; // Distance from buoy divided by pixel width of buoy
const int SERVO_MAX_POSITION = 170; // Maximum angle that servos can output
const int FIN_FORWARD_ANGLE = 10; // Left fin servo value for going forward (right is reversed)
const int FIN_TURN_ANGLE = 170; // Left fin servo value for turning (right is reversed)
const int TUBE_ZERO_ANGLE = 0; // Left tube servo default position for going forward (right is reversed)
const int TURNING_ANGLE = 170; // Angle output for initiating a turn, cutoff for applying valves and fins
const bool TURN_TUBES = true; // Whether to use tube servos for steering
const bool TURN_VALVES = true; // Whether to use valves for steering
const bool TURN_FINS = true; // Whether to use fins for steering

// Pins
const int FIN1 = 3; //right fin
const int FIN2 = 9; //left fin
const int TUBE1 = 10; //right tube pull
const int TUBE2 = 11; //left tube pull
const int VALVE2 = 12; //left valve through relay
const int STOP = 4; //magnetic sensor pin to determin eStop
const int PUMPE = 4; //pump PLL speed control pin
const int PUMPM = 5; //pump motor plug
const int VALVE1E = 7; //valve PLL speed control pin
const int VALVE1M = 6; //valve motor plug


// Objects
Pixy pixy; //creates PixyCam object to use
Servo rightFin, leftFin, leftTube, rightTube;
EasyTransfer ETin, ETout; 
SoftwareSerial XBee(2, 3); // RX, TX

// State variables
int direction = 0; // Computed direction to travel
int *mission[MAX_MISSION_LENGTH]; // Ordered array of targets, e.g. {RED, YELLOW, WHITE, HOME, NONE}
int target = 0; // Current target index
int distance = 0; // Distance from target in inches
int angle = 0; // Angle towards target in degrees CCW
long previousMillis = 0; // Previous loop time in milliseconds
boolean eStop, flood, temp = false; // eStop activated, hull flooding, electronics overheating

// Serial send/recieve structures
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  uint16_t blocks;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  uint16_t blocks;
};

// Give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

// SETUP ROBOT CODE (RUN ONCE) SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  Serial.begin(9600);
  XBee.begin(9600);
  //Serial transfer initialization
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  AFMS.begin();
  
  pixy.init();
  
  rightFin.attach(FIN1);
  leftFin.attach(FIN2);
  rightTube.attach(TUBE1);
  leftTube.attach(TUBE2);

  pinMode(PUMPM, OUTPUT);
  pinMode(VALVE1M, OUTPUT); 
  pinMode(VALVE2, OUTPUT); //left
  pinMode(STOP, INPUT);
  
  digitalWrite(STOP, HIGH);
  
  attachInterrupt(digitalPinToInterrupt(STOP), eStop, LOW);

  systemCheck();

  //delay before start code
  wait(5000);
}


// ROBOT CONTROL LOOP (RUNS UNTIL STOP) LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  digitalWrite(PUMPM, HIGH);
  analogWrite(PUMPE, 255);
  
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    XBee.write(Serial.read());
    mission = XBee.read(); //Probably not right!
  }
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(XBee.read());
  }
  
  downloadMission();
  readSenseArduino();
  think();
  act();
}


// CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC


//delay loop
void wait(int t){
  previousMillis = millis();
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > t) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;
  }
}

// Check for new mission over Serial
void downloadMission() {
  // TODO
  // Overwrite the targets array and set target=0 if a new mission exists
  for (int i=0;i<MAX_MISSION_LENGTH;i++){
    targets[i] = NONE;
  }
  target = 0;
}

// Compute distance and direction from sense Arduino input
void readSenseArduino() {
  // TODO: get an array called "blocks" over Serial, with length "n"
  distance = -1;
  angle = 0;

  //Not sending any data to other arduino, just recieving, right?
  //If sending as well, needs to edit this
  ETin.receiveData(); //recieves data: blocks

  previousMillis = millis();
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > 10) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;
  }

  for (int i=0; i<n; i++) {
    if (blocks[i].signature==mission[target]-2) { // 1, 2, or 3
      distance = CAMERA_RATIO*blocks[i].width;
      angle = blocks[i].x-159; // 159 = center of screen
    }
  }
}

//Check all systems
void systemCheck(){
  rightFin.write(90);
  leftFin.write(90);
  leftTube.write(90);
  rightTube.write(90);
  digital.write(VALVE2, HIGH);
  digitalWrite(PUMPM, HIGH);
  analogWrite(PUMPE, 255);
  digitalWrite(VALVE1M, HIGH);
  analogWrite(VALVE1E, 255);

  previousMillis = millis();
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > 100) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;
  }

  rightFin.write(0);
  leftFin.write(0);
  leftTube.write(0);
  rightTube.write(0);
  digital.write(VALVE2, LOW);
  digitalWrite(PUMPM, LOW);
  analogWrite(PUMPE, 0);
  digitalWrite(VALVE1M, LOW);
  analogWrite(VALVE1E, 0); 
}

//eStop function to shut off all motors
void eStop(){
  eStop = true;
  rightFin.write(0);
  leftFin.write(0);
  leftTube.write(0);
  rightTube.write(0);
  digital.write(VALVE2, LOW);
  digitalWrite(PUMPM, LOW);
  analogWrite(PUMPE, 0);
  digitalWrite(VALVE1M, LOW);
  analogWrite(VALVE1E, 0); 
}


// THINK TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
void think() {
  if (mission[target]<=2) { // Manual override
    direction = mission[target];
  } else if mission[target]==DANCE) { // Dance code
    direction = DANCE;
  } else if (distance<0) { // Target not visible
    direction = LEFT;
  } else if (distance<APPROACH_DIST) { // Reached target
    target++;
    if(mission[target]==LOOP) { // Restart mission
      target=0;
    }
    direction = NONE;
  } else { // Target visible
    direction = STRAIGHT;
  }
}


// ACT AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
void act() {
  switch (direction) {
    case STRAIGHT: // Swim straight using proportional feedback control
      int angleOut = int(K_P*angle);
      move(FORWARD_VELOCITY, int(K_P*angle));
      break;
    case LEFT: // Turn left
      move(TURNING_VELOCITY, TURNING_ANGLE);
      break;
    case RIGHT: // Turn right
      move(TURNING_VELOCITY, -TURNING_ANGLE);
      break;
    case DANCE: // Show off your moves
      break;
    default: // Stop
      move(0, 0);
      break;
  }
}

// Output motor values
void move(int vel, int ang){
  // Set pump output
  digitalWrite(PUMPM, HIGH);
  analogWrite(PUMPE, 255);
  if(vel>0) {
    digitalWrite(PUMPM, HIGH);
  analogWrite(PUMPE, 255);
  } else {
    digitalWrite(PUMPM, LOW);
    analogWrite(PUMPE, 0);
  }
  // Set tube angles
  if(TURN_TUBES) {
    int leftTubeAngle = min(max(TUBE_ZERO_ANGLE+ang, TUBE_ZERO_ANGLE), TUBE_ZERO_ANGLE+TURNING_ANGLE);
    int rightTubeAngle = min(max(TUBE_ZERO_ANGLE-ang, TUBE_ZERO_ANGLE), TUBE_ZERO_ANGLE+TURNING_ANGLE);
    leftTube.write(leftTubeAngle);
    rightTube.write(SERVO_MAX_POSITION-rightTubeAngle);
  } else {
    leftTube.write(TUBE_ZERO_ANGLE);
    rightTube.write(SERVO_MAX_POSITION-TUBE_ZERO_ANGLE);
  }
  // Set fin angles
  if(TURN_FINS && ang>=TURNING_ANGLE) { // left
    leftFin.write(FIN_TURN_ANGLE);
    rightFin.write(FIN_FORWARD_ANGLE);
  } else if(TURN_FINS && ang<=-TURNING_ANGLE) { // right
    leftFin.write(FIN_FORWARD_ANGLE);
    rightFin.write(FIN_TURN_ANGLE);
  } else { // straight
    leftFin.write(FIN_FORWARD_ANGLE);
    rightFin.write(FIN_FORWARD_ANGLE);
  }
  // Set valve states
  if(TURN_VALVES && ang>=TURNING_ANGLE) { // left
    digitalWrite(VALVE1M, LOW);
    analogWrite(VALVE1E, 0);
    digitalWrite(VALVE2, HIGH);
  } else if(TURN_VALVES && ang<=-TURNING_ANGLE) { // right
    digitalWrite(VALVE1M, HIGH);
    analogWrite(VALVE1E, 255);
    digitalWrite(VALVE2, LOW);
  } else { // straight
    digitalWrite(VALVE1M, HIGH);
    analogWrite(VALVE1E, 255);
    digitalWrite(VALVE2, HIGH);
  }
}
