#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "PID_v1.h"
#include <ros.h> 
#include <stdlib.h> 
#include <std_msgs/Int32.h>

#define PWM_R 5
#define PWM_L 6

#define ENCL_B 7 
#define ENCL_A 3 

#define ENCR_B 4 
#define ENCR_A 2 
#define IN4 9
#define IN3 8
#define IN2 11
#define IN1 10


boolean Direction_R, Direction_L;
volatile double duration_R = 0, duration_L = 0;
double abs_duration_R = 0, abs_duration_L = 0;//the number of the pulses
boolean result_R, result_L;

double val_output_R= 0.0, val_output_L=0.0;//Power supplied to the motor PWM value.
double R_pwm_val = 50.0,  L_pwm_val= 50.0;
double Kp_R=0.6, Ki_R=5, Kd_R=0;
double Kp_L=0.6, Ki_L=5, Kd_L=0;


PID myPID_R(&abs_duration_R, &val_output_R, &R_pwm_val, Kp_R, Ki_R, Kd_R, DIRECT);
PID myPID_L(&abs_duration_L, &val_output_L, &L_pwm_val, Kp_L, Ki_L, Kd_L, DIRECT);

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

   myPID_R.SetMode(AUTOMATIC);//PID is set to automatic mode
   myPID_R.SetSampleTime(100);//Set PID sampling frequency is 100ms
   myPID_R.SetOutputLimits(0, 255);

   myPID_L.SetMode(AUTOMATIC);//PID is set to automatic mode
   myPID_L.SetSampleTime(100);//Set PID sampling frequency is 100ms
   myPID_L.SetOutputLimits(0, 255);
  
   //Initialize Direction
   Direction_R = true;//default -> Forward
   Direction_L = true;//default -> Forward
   
   //Initialize the Encoder Interupt
   attachInterrupt(0,readEncoder_R, RISING);
   attachInterrupt(1,readEncoder_L, RISING);
}

void loop() {

  
  result_R = myPID_R.Compute();//PID conversion is complete and returns 1
  result_L = myPID_L.Compute();//PID conversion is complete and returns 1
  
  abs_duration_R=abs(duration_R);
  abs_duration_L=abs(duration_L);

  setMotor(1, val_output_R, PWM_R, IN1, IN2);
  setMotor(1, val_output_L, PWM_L, IN3, IN4);
  Serial.print("duration_L : ");
  Serial.println(duration_L);
  Serial.print("val_output_L: ");
  Serial.println(val_output_L);
  
  
  if (result_R){
      duration_R = 0;
  }
  

  if (result_L){
      duration_L = 0;
  }
  
  delay(300);

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

void readEncoder_R(){
  int b = digitalRead(ENCR_B);
  if(b > 0){
    duration_R++;
  }
  else{
    duration_R--;
  }
}

void readEncoder_L(){
  int b = digitalRead(ENCL_B);
  if(b > 0){
    duration_L++;
  }
  else{
    duration_L--;
  }
}
