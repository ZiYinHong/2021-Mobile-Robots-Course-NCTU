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
//#define PIN_LIGHT 17
#define PIN_LIGHT A0
#define PIN_TOUCH_1 13   //left
#define PIN_TOUCH_2 12   //right
#define PIN_TOUCH_3 19   //down


// arduino get message from raspberry pi
int touch1_val=1, touch2_val=1, touch3_val=1;  //volatile for interupt
int start_val=0;


ros::NodeHandle nh;


void messageCb_start(const std_msgs::Int32 &msg){
  start_val = msg.data;
  char INFO[20];
  sprintf(INFO, "start_val : %d", start_val);
  nh.loginfo(INFO);
}

ros::Subscriber<std_msgs::Int32> arduino_start("topic_rpisend_start",&messageCb_start);

void setup()
{  
   nh.initNode();
   nh.subscribe(arduino_start);

  //  pinMode(ENCR_A,INPUT);
  //  pinMode(ENCR_B,INPUT);
  //  pinMode(ENCL_A,INPUT);
  //  pinMode(ENCL_B,INPUT);

   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);

   pinMode(PWM_R, OUTPUT);
   pinMode(PWM_L, OUTPUT);
   
   pinMode(PIN_LIGHT, INPUT);
   pinMode(PIN_TOUCH_1, INPUT);
   pinMode(PIN_TOUCH_2, INPUT);
   pinMode(PIN_TOUCH_3, INPUT);

}


void loop()
{  
   nh.spinOnce();
  
  int starttime, endtime;
  
  touch3_val = digitalRead(PIN_TOUCH_3);
  
  if (start_val == 1){
//    nh.loginfo("start");
    
    if (touch3_val == 1){ //touch3 not touch
      
      starttime = millis();
      endtime = starttime;
      while ((endtime - starttime) <= 6000) // do this loop for up to 6000mS
      {
        touch1_val = digitalRead(PIN_TOUCH_1);
        touch2_val = digitalRead(PIN_TOUCH_2);
        avoid_obstacle();
        
        endtime = millis();
      }
           
      nh.spinOnce();      
      searching_light();
    }
    else{
      drive(0, 0, 0, 0);   //stop
      delay(5000);
      //exit(0);
    }
  }
  delay(10);
}



void rightMotor(int direc, int motorSpeed)                       
{
  analogWrite(PWM_R, motorSpeed);
  // If speed is positive, run the motor forward
  if (direc > 0) {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
  // If it's negative, run the motor backward
  } else if (direc < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  // If it's 0, brake the motor
  }
 else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }


}



void leftMotor(int direc, int motorSpeed)
{
  analogWrite(PWM_L, motorSpeed);
  // If speed is positive, run the motor forward
  if (direc > 0) {
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
  // If it's negative, run the motor backward
  } else if (direc < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }// If it's 0, brake the motor
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}



void drive(int direction_L, int leftSpeed, int direction_R, int rightSpeed) {
  leftMotor(direction_L, leftSpeed);
  rightMotor(direction_R, rightSpeed);
}



void avoid_obstacle(){  
  if (touch1_val == 0){  //left touch
    nh.loginfo("left touch");
    drive(-1, 80, -1, 80);     //backward
    delay(1700);
    drive(1, 90, 1, 0);       //right
    delay(1600);
    drive(1, 90, 1, 80);     //forward
    delay(3000);
    touch1_val == 1;
    nh.spinOnce();
  }
  else if (touch2_val == 0){   //right touch
    nh.loginfo("right touch");
    drive(-1, 80, -1, 80);     //backward
    delay(1700);
    drive(1, 0, 1, 80);       //left
    delay(1600);
    drive(1, 90, 1, 80);     //forward
    delay(3000);
    touch2_val == 1;
    nh.spinOnce();
  }
  else if (touch1_val == 0 && touch2_val == 0){   //both touch
    drive(-1, 80, -1, 80);     //backward
    nh.loginfo("both touch");
    delay(1700);
    drive(1, 90, 1, 0);       //right
    delay(1600);
    drive(1, 90, 1, 80);     //forward
    delay(3500);
    
    touch1_val == 1;
    touch2_val == 1;
    nh.spinOnce();
  }
  else{
      nh.loginfo("no touch,forward");
      drive(1, 100, 1, 95);    //forward
      nh.spinOnce();
  }
}



void searching_light(){
  int min_light_level=1023;
  int light_level, light_level2;
  int starttime, endtime;
  char INFO[20], INFO2[20];

  // spin first time
  starttime = millis();
  endtime = starttime;
  while ((endtime - starttime) <=8100) // do this loop for up to 8500mS
  {
    drive(1, 0, 1, 80);  

    light_level = analogRead(PIN_LIGHT);  
    sprintf(INFO, "light_level : %d", light_level);
    nh.loginfo(INFO);
    
    if  (light_level < min_light_level){
        min_light_level = light_level;
    }
    endtime = millis();
    
  } 

  sprintf(INFO2, "min_light_level : %d", min_light_level);
  nh.loginfo(INFO2);

  // spin second time
  nh.loginfo("Start second round");
  do{
    drive(1, 0, 1, 80);   
    light_level2 = analogRead(PIN_LIGHT);
  } while (abs(light_level2 - min_light_level) > 15); //Find light
  nh.loginfo("Find light");
  
  drive(0, 0, 0, 0);
  delay(500);
  drive(1, 0, -1, 80);  //迴轉一點
  delay(900);                                                                
  nh.spinOnce();
  drive(1, 95, 1, 80);
  delay(2500);
}
  