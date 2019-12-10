#include "pins.h"


#include <VL53L0X.h>
#include <Wire.h>

//for lab 5
// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = false;
const boolean INVERT_MOTOR_RIGHT = true;

// Loop count, used for print statements
int count = 0;

// Sensor states
float velocity_angular = 0;
float velocity_linear = 0;
float left_dist;
float right_dist;
float center_dist;

// initial power values
float left_power = 0;
float right_power = 0;

void setup() {
  Serial.begin(9600);
  hardwareSetup();
}

void loop() {
  // Read sensor data
  left_dist = getDistanceLeft();
  right_dist = getDistanceRight();
  center_dist = getDistanceCenter();

  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity();
 
  PIDUpdate();
  
  checkEncodersZeroVelocity();
  updateDistanceSensors();
}
