// CONSTANTS
enum {RIGHT=-1, NONE=0, LEFT=1, STRAIGHT=2}; // Directions
enum {RED=3, YELLOW=4, WHITE=5, HOME=6}; // Targets
const int MIN_DIST = 10; // Distance from target to start turning, in inches
const float K_P = 1.0; // Proportional constant for feedback control
const int VELOCITY = 100; // Pump output when moving

// STATE VARIABLES
int direction = 0; // Computed direction to travel
int *targets[5]; // Ordered array of targets
int target = 0; // Current target index
int distance = 0; // Distance from target in inches
int angle = 0; // Angle towards target in degrees CCW

void setup() {
  // TODO
}

void loop() {
  parseCommandData();
  sense();
  think();
  act();
}

// Check Serial for instructions
void parseCommandData() {
  // TODO
  targets[0] = 0;
  target = 0;
}

// Convert sense data into distance and angle from current target
// If target not visible, set distance to -1
void sense() {
  // TODO
  distance = -1;
  angle = 0;
}

// Compute the current robot state
void think() {
  if (targets[target]<=2) { // manual override
    direction = targets[target];
  } else if (distance<0) { // target not visible
    direction = LEFT;
  } else if (distance<MIN_DIST) { // reached target
    target++;
    direction = NONE;
  } else { // target visible
    direction = STRAIGHT;
  }
}


