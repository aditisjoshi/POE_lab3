#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Creating the two motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

// Creating the sensor objects
int leftSensor = A0;
int rightSensor = A1;

// for incoming serial data
int incomingByte;   

// Our variables
// the approximate value for which the sensor is changing from the tape to the floor tiles
int groundTapeDiff = 600;
// straight speed when both sensors are above the tape
int straightSpeed = 40;
// the speed when one wheel is turning
int turnSpeed = straightSpeed/2;
// the speed that the wheels turn when both sensors are above the floor tiles (when left)
int offTurnSpeed = turnSpeed + 5;
// same as above except for right turns because right motor moves more slowly 
int rightOffTurnSpeed = offTurnSpeed + 5;

// if the robot is above the  floor, then it will sweep the same way it was turning, then the opposite, then the same, then the opposite
// this tells it whether or not to switch directions 
boolean switch_direction[] = {true, false, true, false};
// records what the last direction it turned was 
int last_direction;
// the amount of time that the robot turns in one direction when it is above the floor (this increases if it doesn't hit the tape this first time)
int time_off_threshold;
// keeps track of how long the robot has been turning in a specific direction
int time_off;
// keeps track of when the robot started turning in a specific direction (this plus time_off is compared to time_off_threshold)
int time_off_start;
// the number of switches between left and right turns the robot does (when over the floor)
int switches;


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

  Serial.print("LEFT SENSOR VAL");
  Serial.println(leftSensorVal);
  Serial.print("RIGHT SENSOR VAL");
  Serial.println(rightSensorVal);

  // the current time
  int t = millis();

  // read if something is coming into the serial port
  while (Serial.available() > 0) {
    incomingByte = Serial.parseInt();
    Serial.print(incomingByte);
    straightSpeed = incomingByte;
    turnSpeed = straightSpeed/2; 
    offTurnSpeed = turnSpeed + 5;
    rightOffTurnSpeed = offTurnSpeed + 5;
  } 


    // checking to see which (if any) sensors are above the tape 
    if ((leftSensorVal > groundTapeDiff) && (rightSensorVal > groundTapeDiff))
    {
      // if both are over the tape setting both motors to go straight
      Serial.println("STRAIGHT");
      time_off = -1;
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(straightSpeed);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(straightSpeed);
      Serial.print("LEFT MOTOR SPEED");
      Serial.println(straightSpeed);
      Serial.print("RIGHT MOTOR SPEED");
      Serial.println(straightSpeed);
    }
    else if ((leftSensorVal > groundTapeDiff) && (rightSensorVal < groundTapeDiff))
    {
      // setting the robot to turn to the left (right motor goes turn speed backwards and the left motor goes turn speed forwards)
      Serial.println("LEFT");
      time_off = -1;
      last_direction = 0;
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(turnSpeed);
      rightMotor->run(BACKWARD);
      rightMotor->setSpeed(turnSpeed);
      Serial.print("LEFT MOTOR SPEED");
      Serial.println(turnSpeed);
      Serial.print("RIGHT MOTOR SPEED NEG");
      Serial.println(turnSpeed);
    }
    else if ((leftSensorVal < groundTapeDiff) && (rightSensorVal > groundTapeDiff))
    {
      // setting the robot to turn to the right (right motor goes turn speed forwards and the left motor goes turn speed backwards)
      Serial.println("RIGHT");
      time_off = -1;
      last_direction = 1;
      leftMotor->run(BACKWARD);
      leftMotor->setSpeed(turnSpeed);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(turnSpeed);
      Serial.print("LEFT MOTOR SPEED NEG");
      Serial.println(turnSpeed);
      Serial.print("RIGHT MOTOR SPEED");
      Serial.println(turnSpeed);
    }
    else if ((leftSensorVal < groundTapeDiff) && (rightSensorVal < groundTapeDiff)) {
      // setting the robot to turn to the right (left motor goes faster)
      //Serial.println("OH NO!");
      if (time_off == -1) {
       time_off_start = t; 
       time_off_threshold = 500;
       switches = 0;
      } 
      time_off = t - time_off_start;
        
      if (last_direction == 0) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(rightOffTurnSpeed);
        rightMotor->run(BACKWARD);
        rightMotor->setSpeed(rightOffTurnSpeed);
        Serial.print("LEFT MOTOR SPEED");
        Serial.println(rightOffTurnSpeed);
        Serial.print("RIGHT MOTOR SPEED NEG");
        Serial.println(rightOffTurnSpeed);
      }
      else if (last_direction == 1) {
        leftMotor->run(BACKWARD);
        leftMotor->setSpeed(offTurnSpeed);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(offTurnSpeed);
        Serial.print("LEFT MOTOR SPEED NEG");
        Serial.println(offTurnSpeed);
        Serial.print("RIGHT MOTOR SPEED");
        Serial.println(offTurnSpeed);
      }

      
      if (time_off >= time_off_threshold) {
        if (switch_direction[switches]) {
          // Serial.println(switch_direction[switches]);
          if (last_direction == 1) { last_direction = 0; } 
          else if  (last_direction == 0) { last_direction = 1; }
          time_off_start = t; //maybe here
        }
        switches = switches + 1;
        if (switches == 4) {
          time_off_threshold = time_off_threshold + 100;
          switches=0;
        }
      }
    }
}
