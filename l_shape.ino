#include <AccelStepper.h>

///////////////////////////////////////////////////////////////////////////
// Team id: HB1132                                                       //
// Members: Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar // 
// Task: 4B                                                              //
//////////////////////////////////////////////////////////////////////////

int v = 400;
// This is the calculated velocity of each wheel for right/left movement of the robot.
// We found this velocity equations from inverse-kinematics (Task 1/2).
int v1 = -v*0.5;
int v2 = -v*0.5;
int v3 = v;

// Max speed of the motor.
int mxSpeed = 1000;
// Motion time for each movement.
unsigned long runTime = 3000;

// Defining motor pins
const int motor1Pin1 = 11; // step pin
const int motor1Pin2 = 9; // dir pin
const int motor2Pin1 = 12;
const int motor2Pin2 = 10;
const int motor3Pin1 = 6; 
const int motor3Pin2 = 8;

AccelStepper motor1(1, motor1Pin1, motor1Pin2);
AccelStepper motor2(1, motor2Pin1, motor2Pin2);
AccelStepper motor3(1, motor3Pin1, motor3Pin2);

void goFor(unsigned long t){
  unsigned long rightTime = millis();
  while (millis() - rightTime < t) {
    // run all three motors at set speed till t ms
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
  } 
    // stop after t ms
    motor1.stop();
    motor2.stop();
    motor3.stop(); 
  }
  
void setup() {
  // setting maximum speed of the motors
  motor1.setMaxSpeed(mxSpeed);
  motor2.setMaxSpeed(mxSpeed);
  motor3.setMaxSpeed(mxSpeed);
  // setting initial speed to 0
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
}

void loop() {
  // Setting speed for Forward Movement
  motor1.setSpeed(v);
  motor2.setSpeed(-v);
  motor3.setSpeed(0);
  // Running wheels for 3 sec.
  goFor(runTime);
  delay(200);
  // Setting speed for Backward Movement
  motor1.setSpeed(-v);
  motor2.setSpeed(v);
  motor3.setSpeed(0);
  // Running wheels for 3 sec.
  goFor(runTime);
  delay(200);
  // Setting speed for Left Movement
  motor1.setSpeed(v1);
  motor2.setSpeed(v2);
  motor3.setSpeed(v3);
  // Running wheels for 3 sec.
  goFor(runTime);
  delay(200);
  // setting speed for Right Movement
  motor1.setSpeed(-v1);
  motor2.setSpeed(-v2);
  motor3.setSpeed(-v3);
  // Running wheels for 3 sec.
  goFor(runTime);
  delay(200);
}
