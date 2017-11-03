/**
 * Sprint 2 Code Outline
 * Think/Act
 * SquidBot
 * Outline of code for sprint 2
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 11/3/2017
 * Version 1
 */

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


// CONSTANTS AND GLOBAL VARIABLES VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// Constants
enum {RIGHT=-1, NONE=0, LEFT=1, STRAIGHT=2}; // Directions
enum {RED=3, YELLOW=4, WHITE=5, HOME=6, DANCE=7, LOOP=8}; // Targets
const int MIN_DIST = 10; // Distance from target to start turning, in inches
const float K_P = 1.0; // Proportional constant for feedback control
const int VELOCITY = 100; // Pump output for normal swimming
const int MAX_MISSION_LENGTH = 10; // Maximum number of targets in a mission

// Pins
const int FIN1 = 3; //right fin
const int FIN2 = 9; //left fin
const int VALVE1 = 10; //right valve
const int VALVE2 = 11; //left valve
const int STOP = 4; //pin to determine estop for interrupt

// Objects
Pixy pixy; //creates PixyCam object to use
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //creates motor shield
Adafruit_DCMotor *pump = AFMS.getMotor(1); //create bilge pump DC motor plugged into motor shield
Servo rightFin, leftFin;

// State variables
int direction = 0; // Computed direction to travel
int *targets[MAX_MISSION_LENGTH]; // Ordered array of targets, e.g. {RED, YELLOW, WHITE, HOME, NONE}
int target = 0; // Current target index
int distance = 0; // Distance from target in inches
int angle = 0; // Angle towards target in degrees CCW
//eStop true if eStop activated
//flood true if hull flooding
//temp true if electronics overheating
boolean eStop, flood, temp = false;


// SETUP ROBOT CODE (RUN ONCE) SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  Serial.begin(9600);
  pixy.init();
  
  rightFin.attach(FIN1);
  leftFin.attach(FIN2);
  rightValve.attach(VALVE1);
  leftValve.attach(VALVE2);

  attachInterrupt(digitalPinToInterrupt(STOP), eStop, CHANGE);

  //systemCheck
  //waitForLaunch

}


// ROBOT CONTROL LOOP (RUNS UNTIL STOP) LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  //eStop
  downloadMission();
  readSenseArduino();
  think();
  act();
}


// CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC

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
  // TODO
  // The current target is mission[target]
  // Set distance and angle based on that target
  // If target isn't visible, distance=-1
  distance = -1;
  angle = 0;
}


// THINK TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
void think() {
  if (mission[target]<=2) { // Manual override
    direction = mission[target];
  } else if mission[target]==DANCE) { // Dance code
    direction = DANCE;
  } else if (distance<0) { // Target not visible
    direction = LEFT;
  } else if (distance<MIN_DIST) { // Reached target
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
  // TODO
  switch (direction) {
    case STRAIGHT: // Swim straight using proportional feedback control
      int angleOut = int(K_P*angle);
      int distOut = VELOCITY;
      break;
    case LEFT: // Turn left
      break;
    case RIGHT: // Turn right
      break;
    case DANCE: // Show off your moves
      break;
    default: // Stop
      break;
  }
}
