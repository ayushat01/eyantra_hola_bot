#include <AccelStepper.h>
///////////////////////////////////////////////////////////////////////////
// Team id: HB1132                                                       //
// Members: Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar // 
// Task: 5A                                                              //
//////////////////////////////////////////////////////////////////////////

// Defining motor pins
const int motor1Pin1 = 11; // step pin
const int motor1Pin2 = 9; // dir pin
const int motor2Pin1 = 12;
const int motor2Pin2 = 10;
const int motor3Pin1 = 6; 
const int motor3Pin2 = 8;

// Individual velocity of each wheel in steps/rev.
float v1=0;
float v2=0;
float v3=0;

String msg = "0";
String temp;

char arr[25];
char * pch;
int vel[3];
int i =0;

AccelStepper motor1(1, motor1Pin1, motor1Pin2);
AccelStepper motor2(1, motor2Pin1, motor2Pin2);
AccelStepper motor3(1, motor3Pin1, motor3Pin2);

void setup() {
    // setting the maximum speed of each motor.
    motor1.setMaxSpeed(1000);
    motor2.setMaxSpeed(1000);
    motor3.setMaxSpeed(1000);
    Serial.begin(115200);
}

void loop() {
    if(Serial.available()){                  //Check if any data is available on Serial
      msg = Serial.readStringUntil('\n');    //Read message on Serial until new char(\n) which indicates end of message. Received data is stored in msg
      strcpy(arr, msg.c_str());
      pch = strtok (arr," ");
      while (pch != NULL)
      {
        temp = pch;
        vel[i] = temp.toInt();
        i++;
        pch = strtok (NULL, " ");
      }
      i =0;

      v1 = vel[0];
      v2 = vel[1];
      v3 = vel[2];
      // Setting the updated speed of each wheel.
      motor1.setSpeed(v1);
      motor2.setSpeed(v2);
      motor3.setSpeed(v3);
      
    }
    // run all three motors.
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();   
}
