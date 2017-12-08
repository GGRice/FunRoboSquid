/**
 * Sprint 2 Code
 * Think/Act
 * SquidBot
 * Mission: Drive straight to buoy, turn in circle, drive to next buoy, etc.,
 *          then back home
 * Team Squid: Aubrey, Diego, Gretchen, Jon, MJ, Paul  
 * 11/14/2017
 * Version 1
 */

// Library for Serial Transfer
#include <EasyTransfer.h>
#include <SoftwareSerial.h>

// Libraries included to use PixyCam
#include <SPI.h>
#include <Pixy.h>

// Library included to use servos
#include<Servo.h>

// Libraries included to use motor and motion shield
#include <Wire.h>

// CONSTANTS AND GLOBAL VARIABLES VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// Constants
enum {RIGHT=-1, NONE=0, LEFT=1, STRAIGHT=2}; // Directions
enum {RED=3, YELLOW=4, WHITE=5, HOME=6, DANCE=7, LOOP=8}; // Targets
const int APPROACH_DIST = 10; // Distance from target to start turning (inches)
const float K_P = 1.0; // Proportional constant for feedback control
const int FORWARD_VELOCITY = 255; // Pump output for normal swimming
const int TURNING_VELOCITY = 255; // Pump output for turning
const int MAX_MISSION_LENGTH = 10; // Maximum number of targets in a mission
const int CAMERA_RATIO = 1; // Distance from buoy divided by pixel width of buoy (inches/pixel)
const int SERVO_MAX_POSITION = 170; // Maximum angle that servos can output
const int FIN_FORWARD_ANGLE = 10; // Left fin servo value for going forward (right is reversed)
const int FIN_TURN_ANGLE = 170; // Left fin servo value for turning (right is reversed)
const int TUBE_ZERO_ANGLE = 0; // Left tube servo default position for going forward (right is reversed)
const int TURNING_ANGLE = 170; // Angle output for initiating a turn, cutoff for applying valves and fins
const int MAX_BLOCKS = 6; // Maximum number of blocks sent from pixycam
const bool TURN_TUBES = true; // Whether to use tube servos for steering
const bool TURN_VALVES = true; // Whether to use valves for steering
const bool TURN_FINS = true; // Whether to use fins for steering

// Pins
const int FIN1 = 3; // Right fin
const int FIN2 = 9; // Left fin
const int TUBE1 = 10; // Right tube pull
const int TUBE2 = 11; // Left tube pull
const int VALVE2 = 2; // Left valve through relay
const int PUMPE = 7; // Pump PLL speed control pin
const int PUMPM = 6; // Pump motor plug
const int VALVE1E = 4; // Valve PLL speed control pin
const int VALVE1M = 5; // Valve motor plug

// Objects
Servo rightFin, leftFin, leftTube, rightTube;
EasyTransfer ETin, ETout; 
SoftwareSerial Arduino(12,13); //communicate with sense Arduino

// State variables
int direction = NONE; // Computed direction to travel
int mission[MAX_MISSION_LENGTH]; // Ordered array of targets, e.g. {RED, YELLOW, WHITE, HOME, NONE}
int target = 0; // Current target index
int distance = 0; // Distance from target in inches
int angle = 0; // Angle towards target in degrees CCW
long previousMillis = 0; // Previous loop time in milliseconds
boolean flood, temp = false; // E-Stop activated, hull flooding, electronics overheating

// Serial send/recieve structures
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float widths[MAX_BLOCKS];
  int signatures[MAX_BLOCKS];
  float positions[MAX_BLOCKS];
  boolean estop;
};


// Give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;

// SETUP ROBOT CODE (RUN ONCE) SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  // Serial transfer initialization
  Serial.begin(9600);
  Arduino.begin(4800);
  ETin.begin(details(rxdata), &Arduino);

  Serial.println("In setup");

  // Pin initialization
  rightFin.attach(FIN1);
  leftFin.attach(FIN2);
  rightTube.attach(TUBE1);
  leftTube.attach(TUBE2);
  pinMode(PUMPM, OUTPUT);
  pinMode(VALVE1M, OUTPUT); // Right
  pinMode(VALVE2, OUTPUT); // Left
  pinMode(STOP, INPUT);

  Serial.println("About to system check");
  systemCheck();
  Serial.println("System check done");
}


// ROBOT CONTROL LOOP (RUNS UNTIL STOP) LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  delay(50);
  
  Serial.println("In loop");
  downloadMission();
  Serial.println("Mission downloaded");
  readSenseArduino();
  Serial.println("Sense Arduino read");
  think();
  Serial.println("Had a nice thought");
  act();
  Serial.println("Acted on it");
  debug();
  Serial.println("Just debugged");
}


// CONTROL FUNCTIONS CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC

// Delay loop
void wait(int t){
  previousMillis = millis();
  while(millis() - previousMillis <= t) {}
}

// Check for new mission over Serial in the format of a string of characters
void downloadMission() {
  Serial.println("In downloadMission");
  int n = Serial.available();
  if(n<1) { // No message available
    return;
  }
  for (int i=0;i<min(n-1,1); i++) {
    // Map input characters to desired targets
    switch(Serial.read()) {
      case '>': return; // Debug message
      case '2': mission[i] = STRAIGHT; break;
      case '1': mission[i] = LEFT; break;
      case '3': mission[i] = RIGHT; break;
      case 'r': mission[i] = RED; break;
      case 'y': mission[i] = YELLOW; break;
      case 'w': mission[i] = WHITE; break;
      case 'h': mission[i] = HOME; break;
      case 'd': mission[i] = DANCE; break;
      case 'l': mission[i] = LOOP; break;
      default: mission[i] = NONE;
    }
  }
  for (int i=n;i<MAX_MISSION_LENGTH;i++){
    mission[i] = NONE;
  }
  target = 0;
}

// Compute distance and direction from sense Arduino input
void readSenseArduino() {
  Serial.println("In readSenseArduino");
  distance = -1;
  angle = 0;
  if(ETin.receiveData()){ //recieves data: n, blocks
    wait(10);
    if(rxdata.estop){
      eStop();
    }
    for (int i=0; i<MAX_BLOCKS; i++) {
      if (rxdata.signatures[i]==mission[target]-2) { // R,Y,W,H = 1,2,3,4
        distance = CAMERA_RATIO*rxdata.widths[i];
        angle = rxdata.positions[i]-159; // 159 = center of screen
      }
    }
  }
}

//Check all systems
void systemCheck(){
  Serial.println("In system check");
  wait(1000);
  move(0, TURNING_ANGLE);
  wait(1000);
  move(0, -TURNING_ANGLE);
  wait(1000);
  move(0, 0);
}

//eStop function to shut off all motors
void eStop(){
  estop = true;
  rightFin.write(0);
  leftFin.write(0);
  leftTube.write(0);
  rightTube.write(0);
  digitalWrite(VALVE2, LOW);
  digitalWrite(PUMPM, LOW);
  analogWrite(PUMPE, 0);
  digitalWrite(VALVE1M, LOW);
  analogWrite(VALVE1E, 0); 
}

// Output current state over Xbee
void debug() {
  Serial.println("In debug");
  Serial.print(">>> Mission: ");
  for (int i=0; i<MAX_MISSION_LENGTH; i++) {
    Serial.print(mission[i]);
  }
  Serial.print(", Target: ");
  Serial.print(target);
  Serial.print(", Direction: ");
  Serial.print(direction);
  Serial.print(", Distance: ");
  Serial.print(distance);
  Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.print(", Flood: ");
  Serial.print(flood);
  Serial.print(", Temp: ");
  Serial.print(temp);
  Serial.print(", E-Stop: ");
  Serial.println(estop);
}


// THINK TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
void think() {
  Serial.println("In think");
  if (mission[target]<=2) { // Manual override
    direction = mission[target];
  } else if (mission[target]==DANCE) { // Dance code
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
  Serial.println("In act");
  if(estop) {
    move(0,0);
    return;
  }
  switch (direction) {
    case STRAIGHT: // Swim straight using proportional feedback control
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
  Serial.println("In move");
  if(vel>0) {
    digitalWrite(PUMPM, HIGH);
    analogWrite(PUMPE, vel);
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
  if(TURN_FINS && ang>=TURNING_ANGLE) { // Left
    leftFin.write(FIN_TURN_ANGLE);
    rightFin.write(FIN_FORWARD_ANGLE);
  } else if(TURN_FINS && ang<=-TURNING_ANGLE) { // Right
    leftFin.write(FIN_FORWARD_ANGLE);
    rightFin.write(FIN_TURN_ANGLE);
  } else { // Straight
    leftFin.write(FIN_FORWARD_ANGLE);
    rightFin.write(FIN_FORWARD_ANGLE);
  }
  // Set valve states
  if(TURN_VALVES && ang>=TURNING_ANGLE) { // Left
    digitalWrite(VALVE1M, LOW);
    analogWrite(VALVE1E, 0);
    digitalWrite(VALVE2, HIGH);
  } else if(TURN_VALVES && ang<=-TURNING_ANGLE) { // Right
    digitalWrite(VALVE1M, HIGH);
    analogWrite(VALVE1E, 255);
    digitalWrite(VALVE2, LOW);
  } else { // Straight
    digitalWrite(VALVE1M, HIGH);
    analogWrite(VALVE1E, 255);
    digitalWrite(VALVE2, HIGH);
  }
}
