// PID CONSTANTS and CONDITIONS
float u_ang = 0;

float kp_ang = 0.5;
float kp_lin = 0.02;
float ki_ang = 0.05;
float ki_lin = 0.0005;
float kp_dist = .1;

float target_ang = 0;
float target_lin = 60;
float target_dist = 20;

boolean choose_left = true;
boolean is_wall_following = true;

float int_ang = 0;
float int_lin = 0;


void reset() {
  target_ang = 0;
  target_lin = 0;
  is_wall_following = false;
  int_ang = 0;
  int_lin = 0;
  PIDUpdate();
}
void PIDUpdate() {
    int parity = 0;
  
  float actual_dist;
  if( choose_left ) { actual_dist = left_dist;  }
  else              { actual_dist = right_dist; }
  if(is_wall_following){
    if(choose_left) {parity = -1;}
    else            {parity = 1; }
  }
 /* if(center_dist < 20.0) {
    target_ang = 1;
    target_lin=0;
    is_wall_following = false;
    delay(1000);
    target_ang = 0;
    target_lin=120;
    is_wall_following = true;
  }*/
 // else {
  float err_dist = target_dist - actual_dist;
  if(err_dist > 40.0) {err_dist = 40.0;}
  if(err_dist < -40.0) {err_dist = -40.0;}
  float u_dist = kp_dist * err_dist;
  
  float target_ang2 = target_ang + parity * u_dist;
  
  //handling for ang. velocity
  
  float err_ang = velocity_angular + target_ang2;
  int_ang += (err_ang) ;
  u_ang = (err_ang) * kp_ang + int_ang * ki_ang;
  
  //handling for lin. velocity
  float err_lin = -velocity_linear + target_lin;
  int_lin += (err_lin) ;
  float u_lin = err_lin * kp_lin + int_lin * ki_lin;
  
  applyPowerLeft(u_lin+u_ang);
  applyPowerRight(u_lin-u_ang);
 // }
}

void GoStraight()
{
  target_lin = 70;
  target_ang = 0;
  //is_wall_following = true;
}

void Turn()
{
  target_lin = 0;
  target_ang = 1;
  is_wall_following = false;
}
void Forward(double dist)
{
   resetEncoderCounts();
   // Change your PID variable so you go straight and wall follow
  GoStraight();
  while((returnRightEncoderDistance() + returnLeftEncoderDistance()) < dist){
    PIDUpdate();
  }
  reset();
}


void TurnDeg(double deg, int dir) //deg for degree, dir for direction. Should be identical to forward but with different PID values and accounting for direction
{
    resetEncoderCounts();
   
   // Change your PID variable so you turn and no wall follow

   double circumference;
   double distTraveled;  

   while(1)   //Replace 1 with a condition, something with circumference and degrees and encoder readings??? 
      {
           PIDUpdate();
      }

}

void identifyWalls(void) {
  /*
  0 path is blocked✅
  1 left only✅
  2 right only✅
  3 forward only
  4 left or right 
  5 left and forward
  6 right and forward
  7 left, right, and forward*/
  float left_limit = 0;
  float right_limit = 0;
  float front_limit = 0;
  float left = getDistanceLeft();
  float right = getDistanceRight();
  float front = getDistanceCenter();

  if(left<left_limit && front<front_limit && right<right_limit){
    return 0;     
  }
  if(right<right_limit && front<front_limit && left>left_limit){
    return 1;
  }
  if(left<left_limit && front<front_limit && right>right_limit){
    return 2;
  }
  if(left<left_limit && right<right_limit && front>front_limit){
    return 3;
  }
  if(front<front_limit && right>right_limit && left>left_limit){
    return 4;
  }
  if(right<right_limit && left>left_limit && front>front_limit){
    return 5;
  }
  if(left<left_limit && front>front_limit && right>right_limit){
    return 6;
  }
  else{
    return 7;
  } 
}

/*

   Some helper functions we've written to simplify your code
   These were all taken from your previous labs!

   Functions that you might find useful:

    void hardwareSetup()            - sets up pins (modes, interrupts, etc)

    void applyPowerLeft()      - applies a power (-1 to 1) to the left wheel
    void applyPowerRight()     - applies a power (-1 to 1) to the right wheel
    void applyBrakeLeft()      - applies a braking force (0 to 1) to the left wheel
    void applyBrakeRight()     - applies a braking force (0 to 1) to the right wheel

    float getLinearVelocity()   - computes and returns our forward velocity (mm/sec)
    float getAngularVelocity()  - computes and returns our turning velocity (rad/sec)

    float getDistanceLeft()    - reads a distance from the left IR sensor (cm)
    float getDistanceRight()   - reads a distance from the right IR sensor (cm)
    float getDistanceCenter()  - reads a distance from the center IR sensor (cm)

*/

///////////////////////////////
// Helper for hardware setup //
///////////////////////////////

void hardwareSetup() {
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_2, OUTPUT);

  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
  pinMode(PIN_ENCODER_RIGHT_A, INPUT);
  pinMode(PIN_ENCODER_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), leftEncoderRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), rightEncoderRisingEdge, RISING);

  Wire.begin();
  configureDistanceSensors();
}
////////////////////////////////////
// Misc variables needed          //
////////////////////////////////////
double RightEncoderCount = 0;
double LeftEncoderCount = 0;

double DistancePerEncoder = 0.1525; //mm per encoder tick


////////////////////////////////////
// Motor driving helper functions //
////////////////////////////////////

// Apply power to a motor
// power = 0 means no power is applied
// power = 1 means 100% power forward
// power = -1 means 100% power backwards

void applyPowerLeft(float power) {
  if (INVERT_MOTOR_LEFT)
    power *= -1;

  if (power > 0) {
    analogWrite(PIN_MOTOR_LEFT_1, constrain(power * 255.0, 0, 255));
    analogWrite(PIN_MOTOR_LEFT_2, 0);
  } else {
    analogWrite(PIN_MOTOR_LEFT_1, 0);
    analogWrite(PIN_MOTOR_LEFT_2, constrain(power * -255.0, 0, 255));
  }
}

void applyPowerRight(float power) {
  if (INVERT_MOTOR_RIGHT)
    power *= -1;

  if (power > 0) {
    analogWrite(PIN_MOTOR_RIGHT_1, constrain(power * 255.0, 0, 255));
    analogWrite(PIN_MOTOR_RIGHT_2, 0);
  } else {
    analogWrite(PIN_MOTOR_RIGHT_1, 0);
    analogWrite(PIN_MOTOR_RIGHT_2, constrain(power * -255.0, 0, 255));
  }
}


// Apply a braking force to the motor
// power = 0 means no braking force
// power = 1 means 100% braking force

void applyBrakeLeft(float power) {
  analogWrite(PIN_MOTOR_LEFT_1, constrain(power * 255.0, 0, 255));
  analogWrite(PIN_MOTOR_LEFT_2, constrain(power * 255.0, 0, 255));
}

void applyBrakeRight(float power) {
  analogWrite(PIN_MOTOR_RIGHT_1, constrain(power * 255.0, 0, 255));
  analogWrite(PIN_MOTOR_RIGHT_2, constrain(power * 255.0, 0, 255));
}

//////////////////////////////
// Encoder helper functions //
//////////////////////////////

float velocity_left = 0; // millimeters/sec
float velocity_right = 0; // millimeters/sec

// Mouse physical parameters; these are used for velocity calculations
const float ENCODER_TICKS_PER_REVOLUTION = 700;
const float WHEELBASE_DIAMETER = 95.0; // mm
const float WHEEL_DIAMETER = 34.0; // mm

// Precomputed constant used for calculating wheel velocities
// (VELOCITY_COEFF / encoder pulse width in microseconds) will give us the linear velocity of a wheel
const float VELOCITY_COEFF = WHEEL_DIAMETER * PI / ENCODER_TICKS_PER_REVOLUTION * 1000000.0;

// Encoder helper variables
unsigned long prev_pulse_time_right;
unsigned long prev_pulse_time_left;

// Encoder state variables
float getLinearVelocity() {
  return 0.5 * (velocity_left + velocity_right);
}

float getAngularVelocity() {
  return (velocity_right - velocity_left) / WHEELBASE_DIAMETER;
}

void checkEncodersZeroVelocity(void) {
  // Sets the wheel velocity to 0 if we haven't see an edge in a while
  unsigned long curr_time = micros();
  if (curr_time > prev_pulse_time_left + 100000) {
    velocity_left = 0;
  }
  if (curr_time > prev_pulse_time_right + 100000) {
    velocity_right = 0;
  }
}
void resetEncoderCounts(void) {
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
}

int getLEC() {
  return LeftEncoderCount;
}
int getREC() {
  return RightEncoderCount;
}
double returnRightEncoderDistance(void)
{
  return DistancePerEncoder * RightEncoderCount;
}

double returnLeftEncoderDistance(void)
{
  return DistancePerEncoder * LeftEncoderCount;
}

void leftEncoderRisingEdge(void) {
  unsigned long curr_time = micros();

  int direction;
  if (digitalRead(PIN_ENCODER_LEFT_A) == !INVERT_ENCODER_LEFT) {
    direction = 1;
    LeftEncoderCount++;
  } else {
    direction = -1;
    LeftEncoderCount--;
  }

  if (direction * velocity_left < 0) {
    velocity_left = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_left = direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_left);
  }

  prev_pulse_time_left = curr_time;
}

void rightEncoderRisingEdge(void) {
  unsigned long curr_time = micros();

  int direction;
  if (digitalRead(PIN_ENCODER_RIGHT_A) == !INVERT_ENCODER_RIGHT) {
    direction = -1;
    RightEncoderCount--;
  } else {
    direction = 1;
    RightEncoderCount++;
  }

  if (direction * velocity_right < 0) {
    velocity_right = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_right = direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_right);
  }

  prev_pulse_time_right = curr_time;
}

////////////////////////////////
// ToF sensor helper functions //
////////////////////////////////

VL53L0X distance_sensor_center;
VL53L0X distance_sensor_left;
VL53L0X distance_sensor_right;

int left_reading;
int right_reading;
int center_reading;

float getDistanceLeft(void) {
  return left_reading / 10.0;
}

float getDistanceRight(void) {
  return right_reading / 10.0;
}

float getDistanceCenter(void) {
  return center_reading / 10.0;
}

long last_distance_reading;

void updateDistanceSensors(void) {
  // Read from the sensors at 10Hz
  if (millis() >= last_distance_reading + 100) {
    left_reading = distance_sensor_left.readRangeContinuousMillimeters();
    right_reading = distance_sensor_right.readRangeContinuousMillimeters();
    center_reading = distance_sensor_center.readRangeContinuousMillimeters();
    last_distance_reading = millis();
  }
}

void configureDistanceSensors(void) {


  // Set up the three distance sensors.
  //       This is be done as follows:
  //       1) Disable the left and right sensors.
  //       2) Set up the center sensor with an address of 0x01
  //       3) Enable the left sensor, and set it up with an address of 0x02
  //       4) Enable the right sensor, and set it up with an address of 0x03
  // IMPORTANT: do *not* write a HIGH signal to a reset pin; this will destroy the ToF sensor

  pinMode(PIN_TOF_LEFT_RESET, OUTPUT);
  digitalWrite(PIN_TOF_LEFT_RESET, LOW);
  pinMode(PIN_TOF_RIGHT_RESET, OUTPUT);
  digitalWrite(PIN_TOF_RIGHT_RESET, LOW);

  // Set up center sensor with address 0x01
  configureDistanceSensor(&distance_sensor_center, 0x01);

  // Set up left sensor with address 0x02
  pinMode(PIN_TOF_LEFT_RESET, INPUT);
  configureDistanceSensor(&distance_sensor_left, 0x02);

  // Set up right sensor with address 0x03
  pinMode(PIN_TOF_RIGHT_RESET, INPUT);
  configureDistanceSensor(&distance_sensor_right, 0x03);



}



void configureDistanceSensor(VL53L0X* sensor, uint8_t addr) {
  delay(100);
  sensor->setAddress(addr);
  sensor->init();
  //sensor->configureDefault();
  //sensor->writeReg(VL53L0X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  //sensor->writeReg16Bit(VL53L0X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor->setTimeout(500);
  //sensor->stopContinuous();
  sensor->startContinuous(100);
}
