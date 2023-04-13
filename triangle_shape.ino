#include <AccelStepper.h>

///////////////////////////////////////////////////////////////////////////
// Team id: HB1132                                                       //
// Members: Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar // 
// Task: 4B                                                              //
//////////////////////////////////////////////////////////////////////////

int v = 500;
// This is the calculated velocity of each wheel for right/left movement of the robot.
// We found this velocity equations from inverse-kinematics (Task 1/2).
int v1 = -v*0.5;
int v2 = -v*0.5;
int v3 = v;

int mxSpeed = 1000;

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
  // Setting velocity of each wheel for 1st edge of the triangle.
  motor1.setSpeed(-500);
  motor2.setSpeed(0);
  motor3.setSpeed(500);
  // commanding steppers to move on 1st edge of the triangle for 2 sec.
  goFor(2000);
  delay(200);
  
  // Setting velocity of each wheel for 2nd edge of the triangle.
  motor1.setSpeed(0);
  motor2.setSpeed(-500);
  motor3.setSpeed(500);
  // commanding steppers to move on 2nd edge of the triangle for 2 sec.
  goFor(2000);
  delay(200);
  
  // Setting calculated velocity of each wheel for base of the triangle.
  motor1.setSpeed(-v1);
  motor2.setSpeed(-v2);
  motor3.setSpeed(-v3);
  // commanding steppers to move on base of the triangle for 3.5 sec.  
  goFor(3500);
  delay(200);
}
