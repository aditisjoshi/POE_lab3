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

int incomingByte = 0;   // for incoming serial data

//bool is_running = false;


// Our variables
int groundTapeDiff = 300;
// when calibrating with a sensor at ~1in from the ground we found that the tape values would be between 20-40 and the ground values would be between 200-300 
int turnSpeed = 20;
int offTurnSpeed = 25;
int rightOffTurnSpeed = 30;
int straightSpeed = 40;
int time_off_threshold = 600;
boolean switch_direction[] = {true, false, true, false};


int last_direction;
int time_off;
int time_off_start;
int switches;


int stop_string = 115;
int start_string = 103;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz

//  is_running = true;
  
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

//  off_direction_order[0] = true;
//  off_direction_order[1] = false;
//  off_direction_order[2] = true;
//  off_direction_order[3] =false;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // reading the sensor values
  int leftSensorVal = analogRead(leftSensor);
  int rightSensorVal = analogRead(rightSensor);
//  Serial.print("leftsensor");
//  Serial.println(leftSensorVal);
//  Serial.print("rightsensor");
//  Serial.println(rightSensorVal);

  int t = millis();

  //read if something is coming into the serial port
//  incomingByte = Serial.read();
//  Serial.println(incomingByte);
//  Serial.println(is_running);

  //g = go = 103
  //s = stop = 115

//  if (incomingByte == stop_string) {
//    is_running = false;
//  }
//  else if (incomingByte == start_string) {
//    is_running = true;
//  }
//  else {
//    is_running = true;
//  }


//  if (is_running) {
    // checking to see which (if any) sensors are above the tape 
    if ((leftSensorVal > groundTapeDiff) && (rightSensorVal > groundTapeDiff))
    {
      // setting both motors to go straight
      Serial.println("STRAIGHT");
      time_off = -1;
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(straightSpeed);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(straightSpeed);
    }
    else if ((leftSensorVal > groundTapeDiff) && (rightSensorVal < groundTapeDiff))
    {
      // setting the robot to turn to the left (right motor goes faster)
      Serial.println("LEFT");
      time_off = -1;
      last_direction = 0;
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(turnSpeed);
      rightMotor->run(BACKWARD);
      rightMotor->setSpeed(turnSpeed);
    }
    else if ((leftSensorVal < groundTapeDiff) && (rightSensorVal > groundTapeDiff))
    {
      // setting the robot to turn to the right (left motor goes faster)
      Serial.println("RIGHT");
      time_off = -1;
      last_direction = 1;
      leftMotor->run(BACKWARD);
      leftMotor->setSpeed(turnSpeed);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(turnSpeed);
    }
    else if ((leftSensorVal < groundTapeDiff) && (rightSensorVal < groundTapeDiff)) {
      // setting the robot to turn to the right (left motor goes faster)
      Serial.println("FUCK");
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
      }
      else if (last_direction == 1) {
        leftMotor->run(BACKWARD);
        leftMotor->setSpeed(offTurnSpeed);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(offTurnSpeed);
      }

      
      if (time_off >= time_off_threshold) {
        if (switch_direction[switches]) {
          Serial.println(switch_direction[switches]);
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

    

//  }
  
//  else {
//    // setting both motors to go straight
//    leftMotor->run(FORWARD);
//    leftMotor->setSpeed(0);
//    rightMotor->run(FORWARD);
//    rightMotor->setSpeed(0);
//  }

 
}
