#include "PID_v1.h"
#include <ros.h> 
#include <stdlib.h> 
#include <std_msgs/Int32.h>

#define PWM_R 5
#define PWM_L 6

#define ENCL_B 7 
#define ENCL_A 3  //0

#define ENCR_B 4 
#define ENCR_A 2  //1
#define IN4 8
#define IN3 9
#define IN2 10
#define IN1 11

byte encoder0PinALast_R, encoder0PinALast_L;
boolean Direction_R, Direction_L;
double duration_R = 0, duration_L = 0;  //the number of the pulses
double abs_duration_R = 0, abs_duration_L = 0; 
boolean result_R, result_L;


double val_output_R, val_output_L;//Power supplied to the motor PWM value.
double Setpoint;
double Kp_R=0.6, Ki_R=5, Kd_R=0;
double Kp_L=0.6, Ki_L=5, Kd_L=0;

boolean flagR = false;
boolean flagL = false;

// arduino get message from raspberry pi
ros::NodeHandle nh;
double R_pwm_val = 0.0, L_pwm_val=0.0;  //volatile for interupt

void messageCb_R(const std_msgs::Int32 &msg){
  int R_pwm;
  R_pwm = msg.data;

  if (R_pwm < 0){
    Direction_R = false;
  }
  else{
    Direction_R = true;
  }

  R_pwm_val = abs(static_cast<double>(R_pwm));
  flagR = true;
}
void messageCb_L(const std_msgs::Int32 &msg){
  int L_pwm;
  L_pwm = msg.data;

  if (L_pwm < 0){
    Direction_L = false;
  }
  else{
    Direction_L = true;
  }

  L_pwm_val = abs(static_cast<double>(L_pwm));
  flagL = true;
}

ros::Subscriber<std_msgs::Int32> R_arduino_sub("topic_rpisend_R",&messageCb_R);
ros::Subscriber<std_msgs::Int32> L_arduino_sub("topic_rpisend_L",&messageCb_L);


PID myPID_R(&abs_duration_R, &val_output_R, &R_pwm_val, Kp_R, Ki_R, Kd_R, DIRECT);
PID myPID_L(&abs_duration_L, &val_output_L, &L_pwm_val, Kp_L, Ki_L, Kd_L, DIRECT);


void setup()
{  
   nh.initNode();
   nh.subscribe(R_arduino_sub);
   nh.subscribe(L_arduino_sub);

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

   //Setpoint =80;  //Set the output value of the PID
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
   attachInterrupt(0, MotorStateChanged_R, RISING);
   attachInterrupt(1, MotorStateChanged_L, RISING);
}

void loop()
{  
    if(flagR && flagL){
      abs_duration_R=abs(duration_R);
      myPID_R.SetTunings(Kp_R, Ki_R, Kd_R);
      result_R = myPID_R.Compute();//PID conversion is complete and returns 1
      if(result_R)
      {
      duration_R = 0; //Count clear, wait for the next count
      }
  
  
      abs_duration_L=abs(duration_L);
      myPID_L.SetTunings(Kp_L, Ki_L, Kd_L);
      result_L = myPID_L.Compute();//PID conversion is complete and returns 1
      if(result_L)
      {
      duration_L = 0; //Count clear, wait for the next count
      }
  
      setMotor(Direction_R, val_output_R, PWM_R, IN1, IN2);
      setMotor(Direction_L, val_output_L, PWM_L, IN3, IN4);
    }

    nh.spinOnce();

}


// For one motor
void setMotor(boolean dir, double pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if (pwmVal != 0){
    if(dir == true){  //CW  //Forward
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    }
    else if(dir == false){  //CCW  //Backward
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    }
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}


void MotorStateChanged_R(){
  int val = digitalRead(ENCR_B);
  if(val == LOW){
      Direction_R = false; //Reverse
      duration_R--;
  }
  else {
      Direction_R = true;  //Forward
      duration_R++;
  }
}


void MotorStateChanged_L(){
  int val = digitalRead(ENCL_B);
  if(val == LOW){
      Direction_L = false; //Reverse
      duration_L--;
  }
  else {
      Direction_L = true;  //Forward
      duration_L++;
  }
}