// NO PID controller
#include <ros.h> 
#include <stdlib.h> 
#include <std_msgs/Int32.h>
#include "motor.h"

ros::NodeHandle nh;
int r_pwm = 0, l_pwm = 0;
Modes mode;
boolean flagR = false;
boolean flagL = false;

//ros::Publisher arduino_pub("topic_arduinosend", &return_msg);

// Motor (ENA, ENB, IN1, IN2, IN3, IN4)
Motor motor(5, 6, 8, 9, 10, 11);

void messageCb_R(const std_msgs::Int32 &R_pwm){
  r_pwm = R_pwm.data ;
  flagR = true;
}
void messageCb_L(const std_msgs::Int32 &L_pwm){
  l_pwm = L_pwm.data ;
  flagL = true;
}

ros::Subscriber<std_msgs::Int32> R_arduino_sub("topic_rpisend_R", messageCb_R);
ros::Subscriber<std_msgs::Int32> L_arduino_sub("topic_rpisend_L", messageCb_L);


void setup() {
  //nh.advertise(arduino_pub);

  nh.initNode(); 
  nh.subscribe(R_arduino_sub);
  nh.subscribe(L_arduino_sub);
  
  motor.pinmode(5, 6, 8, 9, 10, 11);
}

//mode : FORWARD, BACKWARD, LEFT, RIGHT, STOP
void loop() {
  //Serial.print(msg.data, return_msg.data);
  if(flagR && flagL){
    flagR = false;
    flagL = false;
    if (r_pwm > 0 && l_pwm > 0){
      mode = FORWARD;
    }
    else if (r_pwm < 0 && l_pwm < 0){
      mode = BACKWARD;
    }
    else if (r_pwm < 0 && l_pwm > 0){
      mode = RIGHT;
    }
    else if (r_pwm > 0 && l_pwm < 0){
      mode = LEFT;
    }
    else{
      mode =  STOP;
    }

    motor.drive(mode, abs(r_pwm), abs(l_pwm));
  } 
  delay(50);

  nh.spinOnce();
}
