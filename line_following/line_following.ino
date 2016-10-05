#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Creating the two motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Creating the sensor objects
int leftSensor = A0;
int rightSensor = A1;

// Our variables
int groundTapeDiff = 100;
// when calibrating with a sensor at ~1in from the ground we found that the tape values would be between 20-40 and the ground values would be between 200-300 
int turnSpeed = 150;
int straightSpeed = 200;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(straightSpeed);
  leftMotor->run(FORWARD);
  // turn on motor
  leftMotor->run(RELEASE);
   // do the same for the right motor
  rightMotor->setSpeed(straightSpeed);
  rightMotor->run(FORWARD);
  // turn on motor
  rightMotor->run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  // reading the sensor values
  int leftSensorVal = analogRead(leftSensor);
  int rightSensorVal = analogRead(rightSensor);
  Serial.println(leftSensorVal);
  Serial.println(rightSensorVal);

  // checking to see which (if any) sensors are above the tape 
  if ((leftSensorVal < 100) && (rightSensorVal < 100))
  {
    // setting both motors to go straight
    leftMotor->setSpeed(straightSpeed);
    leftMotor->run(FORWARD);
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(straightSpeed);
    rightMotor->run(FORWARD);
    rightMotor->run(RELEASE);
  }
  else if ((leftSensorVal < 100) && (rightSensorVal > 100))
  {
    // setting the robot to turn to the left (right motor goes faster)
    leftMotor->setSpeed(straightSpeed);
    leftMotor->run(FORWARD);
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(turnSpeed);
    rightMotor->run(FORWARD);
    rightMotor->run(RELEASE); 
  }
  else if ((leftSensorVal > 100) && (rightSensorVal < 100))
  {
    // setting the robot to turn to the right (left motor goes faster)
    leftMotor->setSpeed(turnSpeed);
    leftMotor->run(FORWARD);
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(straightSpeed);
    rightMotor->run(FORWARD);
    rightMotor->run(RELEASE); 
  };
 
}
