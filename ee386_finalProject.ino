/*
EE 386 - Intro to Controls Systems and Robotics Final Project
Single Axis Ball-in-Tray Stabilizer

Author: Nick Polickoski
File Creation: 03/27/2025

Info:
- General Info: https://docs.arduino.cc/hardware/uno-rev3/#tech-specs
- Ultra Sonic Shite: https://projecthub.arduino.cc/lucasfernando/ultrasonic-sensor-with-arduino-complete-guide-284faf
- Servo Shite: https://projecthub.arduino.cc/arduino_uno_guy/the-beginners-guide-to-micro-servos-ae2a30
- Example: https://electronoobs.com/eng_arduino_tut100_code1.php

Log:
- 03/27/2025  - Stole Ultrasonic sensor code from online and changed
                it to make estimate for project
              - Stole servo motor code from online
              - 
*/

// Libraries
#include <Servo.h>


// Global Variables
float timeElapsed = 0.0, US_reading = 0.0;
float period = 50;                                  // (ms)
Servo servo;
const int USPIN_ECHO = 9;
const int USPIN_TRIGGER = 10;
const int SERVOPIN = 8;
const int SERVO_INITVAL = 90;                       // (degrees)

// PID Controller Variables
float Kp = 0, Ki = 0, Kd = 1000;
float ErrTotal, ErrP, ErrI, ErrD;
float distanceDesired = 14.5;                       // middle of tray (cm)
float distanceCurrent = 0.0, distanceError = 0.0, distanceErrorPrev = 0.0;

// P = 0.25
// I = 0.04
// D = 25

// Functions
void setup()
{
  Serial.begin(9600);
  // Set GPIO Pins
  pinMode(USPIN_ECHO, INPUT);
  pinMode(USPIN_TRIGGER, OUTPUT);
  servo.attach(SERVOPIN);

  // Initial Writes
  digitalWrite(USPIN_TRIGGER, LOW);
  servo.write(SERVO_INITVAL);
  timeElapsed = millis();                           // start of system time (ms)
}


void loop()
{
  // Sensor
  digitalWrite(USPIN_TRIGGER, LOW);
  delay(2);
  digitalWrite(USPIN_TRIGGER, HIGH);
  delay(10);
  digitalWrite(USPIN_TRIGGER, LOW);
  US_reading = pulseIn(USPIN_ECHO, HIGH);


  // Distance Error Value Calculations
  distanceCurrent = (US_reading * 0.034) / 2;              // converts speed of sound to distance (cm)
  if (distanceCurrent > 30.0)
  {
    distanceCurrent = 30.0;
  }
  else if (distanceCurrent < 2.00)
  {
    distanceCurrent = 2.00;
  }
  Serial.print("Current: ");
  Serial.print(distanceCurrent);
  distanceError = distanceDesired - distanceCurrent;
  Serial.print(" Error: ");
  Serial.print(distanceError);
  // Serial.print("\n");

  // Controller Values
  ErrP = Kp*distanceError;
  ErrD = Kd*((distanceError - distanceErrorPrev) / period);
  Serial.print(" ErrD: ");
  Serial.print(ErrD);
  Serial.print("\n");
  if (distanceCurrent < 2.0 || distanceCurrent > 30.0)
  {
    ErrI = 0;
  }
  else
  {
    ErrI = ErrI + Ki*distanceError;
  }


  // Actuator
  ErrTotal = ErrP + ErrI + ErrD;
  //ErrTotal = static_cast<int>(ErrTotal);
  ErrTotal = map(ErrTotal, -15, 15, 180, 0);
  servo.write(ErrTotal);


  // End of Cycle
  distanceErrorPrev = distanceError;
  delay(100);
}



// // SERVO TESTING
// Servo servo;

// void setup()
// {
//   servo.attach(8);
// }

// void loop()
// {
//   for (int i=0; i<180; i++)
//   {
//     servo.write(i);
//     delay(15);
//   }
//   for (int i=180; i>0; i--)
//   {
//     servo.write(i);
//     delay(15);
//   }
// }


