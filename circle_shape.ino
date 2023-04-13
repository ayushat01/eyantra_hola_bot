#include <AccelStepper.h>
///////////////////////////////////////////////////////////////////////////
// Team id: HB1132                                                       //
// Members: Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar // 
// Task: 4B                                                              //
//////////////////////////////////////////////////////////////////////////

// Defining motor pins
const int motor1Pin1 = 11; // step pin
const int motor1Pin2 = 9; // dir pin
const int motor2Pin1 = 12;
const int motor2Pin2 = 10;
const int motor3Pin1 = 6; 
const int motor3Pin2 = 8;

float t=0; // Elapsed time.
float ra=2; // Radius of the circle in meters.
float V = 7; // Linear velocity of the robot.

// Individual velocity of each wheel in steps/rev.
float v1=0;
float v2=0;
float v3=0;

AccelStepper motor1(1, motor1Pin1, motor1Pin2);
AccelStepper motor2(1, motor2Pin1, motor2Pin2);
AccelStepper motor3(1, motor3Pin1, motor3Pin2);

void setup() {
    // setting the maximum speed of each motor.
    motor1.setMaxSpeed(1000);
    motor2.setMaxSpeed(1000);
    motor3.setMaxSpeed(1000);
    
    cli();//stop interrupts
      //set timer1 interrupt at 1kHz
      TCCR1A = 0;// set entire TCCR1A register to 0
      TCCR1B = 0;// same for TCCR1B
      TCNT1  = 0;//initialize counter value to 0
      // set compare match register for 1hz increments.
      OCR1A = 15;// = (16*10^6) / (1*1024) - 1.
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // Set CS12 and CS10 bits for 1024 prescaler
      TCCR1B |= (1 << CS12) | (1 << CS10);  
      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect){
    // Update each wheel velocity 
    t = (t + 0.3);
    // for circular motion 
    // Xw = Xo + ra*cos(w*t), Yw = Yo + ra*sin(wt), phiw = 0
    // Using above equations, these are the individual velocity equations.
    // which comes from inverse-kinematics.
    v1 = - ra*V*cos((V*t)/(ra*1000))/2 + 1.73/2*ra*V*sin((V*t)/(ra*1000));
    v2 = - ra*V*cos((V*t)/(ra*1000))/2 - 1.73/2*ra*V*sin((V*t)/(ra*1000));
    v3 = ra*V*cos((V*t)/(ra*1000));
}

void loop() {
    // Setting the updated speed of each wheel.
    motor1.setSpeed(v1*30);
    motor2.setSpeed(v2*30);
    motor3.setSpeed(v3*30);
    // run all three motors.
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();   
}
