#include <ros.h> 
#include <stdlib.h> 
#include <std_msgs/Int32.h>

#define IN4 9
#define IN3 8
#define IN2 11
#define IN1 10
#define PWM_R 5
#define PWM_L 6
#define ENCL_B 7 
#define ENCL_A 3 

#define ENCR_B 4 
#define ENCR_A 2 
#define LIGHT_PIN A0

int light_level;

void setup() {
  Serial.begin(57600);
  
   pinMode(ENCR_A,INPUT);
   pinMode(ENCR_B,INPUT);
   pinMode(ENCL_A,INPUT);
   pinMode(ENCL_B,INPUT);
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
   //L298P Control port settings DC motor driver board for the output mode
   pinMode(PWM_R, OUTPUT);
   pinMode(PWM_L, OUTPUT);

}

void loop(){
    drive(0, 0, 1, 60);
    light_level = analogRead(LIGHT_PIN);
    Serial.print("light_level : ");
    Serial.println(light_level);
}

void rightMotor(int direction, int motorSpeed)                       
{
  // If speed is positive, run the motor forward
  if (direction > 0) {
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  // If it's negative, run the motor backward
  } else if (direction < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  // If it's 0, brake the motor
  }
 else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(PWM_R, motorSpeed);
}



void leftMotor(int direction, int motorSpeed)
{
  // If speed is positive, run the motor forward
  if (direction > 0) {
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
  // If it's negative, run the motor backward
  } else if (direction < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }// If it's 0, brake the motor
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  
  analogWrite(PWM_R, motorSpeed);
}


void drive(int direction_L, int leftSpeed, int direction_R, int rightSpeed) {
  leftMotor(direction_L, leftSpeed);
  rightMotor(direction_R, rightSpeed);
}