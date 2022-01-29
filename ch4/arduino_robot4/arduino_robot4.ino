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
#define IR_PIN 2


// arduino get message from raspberry pi
int touch1_val=1, touch2_val=1, touch3_val=1;  //volatile for interupt
int start_val=0;
volatile int IR_value;
int door_num;
int find_door_flag;

ros::NodeHandle nh;

const int interruptNumber = 0;           // Interrupt 0 在 pin 2 上


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
   pinMode(IR_PIN,INPUT);

   //attachInterrupt(interruptNumber, IRChange, RISING);
}


void loop()
{  
   nh.spinOnce();
  
  int starttime, endtime;
  int result;
  char INFO[20];
  
  touch3_val = digitalRead(PIN_TOUCH_3);
  
  if (start_val == 1){ 
    if (touch3_val == 1){ //touch3 not touch
      
      starttime = millis();
      endtime = starttime;
      while ((endtime - starttime) <= 5500) 
      {
        touch1_val = digitalRead(PIN_TOUCH_1);
        touch2_val = digitalRead(PIN_TOUCH_2);
        avoid_obstacle();
        
        touch3_val = digitalRead(PIN_TOUCH_3);
        if (touch3_val == 0){
          break;
        }
        
        endtime = millis();
      }
           
      nh.spinOnce();
      if (touch3_val == 1){
          searching_light();
      }      
    }
    else{                  //touch3 touch, get the light ball, Goal door Seeking 
      delay(100); 
      nh.spinOnce();
      nh.loginfo("start searching door");    
      result = searching_door();        // spin one round & move forward
      
      nh.spinOnce();
      if (result == 1){
        nh.loginfo("find door!!!!!!");
        drive(0, 0, 0, 0);
        delay(500);
        
        
        starttime = millis();
        endtime = starttime;
        while ((endtime - starttime) <=3000) 
        {
          drive(1, 105, 1, 100);     //forward
  
          endtime = millis();
        } 
     }
     else{
        starttime = millis();
        endtime = starttime;
        while ((endtime - starttime) <=1300)
        {
          avoid_obstacle_door();                  //move forward
          
          nh.spinOnce();
          endtime = millis();
        } 
     }     
   }
 }
 nh.spinOnce();
 delay(10);
}


//void IRChanged() {
//  IR_value = digitalRead(IR_PIN);
//  digitalWrite(ledPin, buttonState);
//}


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
    drive(-1, 105, -1, 100);     //backward
    delay(1700);
//    drive(0, 0, 0, 0);      
//    delay(100);
    drive(1, 105, 1, 0);       //right
    delay(1000);
    drive(1, 105, 1, 100);     //forward
    delay(3200);
    touch1_val == 1;
    nh.spinOnce();
  }
  else if (touch2_val == 0){   //right touch
    nh.loginfo("right touch");
    drive(-1, 105, -1, 100);     //backward
    delay(1700);
//    drive(0, 0, 0, 0);      
//    delay(100);
    drive(1, 0, 1, 105);       //right
    delay(1000);
    drive(1, 105, 1, 100);     //forward
    delay(3200);
    touch2_val == 1;
    nh.spinOnce();
  }
  else if (touch1_val == 0 && touch2_val == 0){   //both touch
    nh.loginfo("both touch");
    drive(-1, 105, -1, 100);     //backward
    delay(1700);
//    drive(0, 0, 0, 0);      
//    delay(100);
    drive(1, 105, 1, 0);       //left
    delay(1000);
    drive(1, 105, 1, 100);     //forward
    delay(3200);
    
    touch1_val == 1;
    touch2_val == 1;
    nh.spinOnce();
  }
  else{
      nh.loginfo("no touch,forward");
      drive(1, 130, 1, 120);    //forward
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
  while ((endtime - starttime) <= 4830) 
  {
    drive(1, 0, 1, 115);  

    light_level = analogRead(PIN_LIGHT);  
//    sprintf(INFO, "light_level : %d", light_level);
//    nh.loginfo(INFO);
    
    if  (light_level < min_light_level){
        min_light_level = light_level;
    }
    
    touch3_val = digitalRead(PIN_TOUCH_3);
    if (touch3_val == 0){
          return;
    }
    endtime = millis();
  } 
  sprintf(INFO2, "min_light_level : %d", min_light_level);
  nh.loginfo(INFO2);

  // spin second time
  nh.loginfo("Start second round");
  nh.spinOnce();
  do{
    drive(1, 0, 1, 115);   
    light_level2 = analogRead(PIN_LIGHT);  
        
    touch3_val = digitalRead(PIN_TOUCH_3);
    if (touch3_val == 0){
          return;
    }
  } while (abs(light_level2 - min_light_level) > 15); //Find light
  nh.loginfo("Find light");
  
  drive(0, 0, 0, 0);
  delay(500);
  drive(1, 0, -1, 95);  //迴轉一點
  delay(900);                                                                
  nh.spinOnce();
  drive(1, 105, 1, 95);
  delay(2800);
}


int calculate_ratio(){
  int zero_count, one_count, total_count;
  float ratio;
  int IR_value;
  char INFO[20], INFO1[20];
  int door_number;
  
  for(int i = 0 ; i<200; i++){
      IR_value = digitalRead(IR_PIN); 
     
      if (IR_value == 1){
         one_count++;
      }
      else{
         zero_count++;
      }
      delay(1);
   }

  total_count = zero_count + one_count;
  ratio = (float)zero_count/(float)total_count;
  sprintf(INFO1, "zero_count : %d, one_count : %d", zero_count , one_count);
  nh.loginfo(INFO1);
  
  sprintf(INFO, "ratio : %.2f", ratio);
  nh.loginfo(INFO);
  
  if (ratio > 0.15 && ratio < 0.3){
     nh.loginfo("goal door 1500");
     door_number = 1500;
  }
  else if (ratio > 0.35 && ratio < 0.5){
    nh.loginfo("goal door 600");
    door_number = 600;
  }
  
  one_count = 0; 
  zero_count = 0;
  
  return door_number;
}


void avoid_obstacle_door(){  
  if (touch1_val == 0){  //left touch
    nh.loginfo("avoid_obstacle_door left touch");
    drive(-1, 80, -1, 80);     //backward
    delay(1000);
    drive(1, 90, 1, 0);       //right
    delay(1500);
    drive(1, 90, 1, 80);     //forward
    delay(1500);
    touch1_val == 1;
    nh.spinOnce();
  }
  else if (touch2_val == 0){   //right touch
    nh.loginfo("avoid_obstacle_door right touch");
    drive(-1, 80, -1, 80);     //backward
    delay(1000);
    drive(1, 0, 1, 80);       //left
    delay(1500);
    drive(1, 90, 1, 80);     //forward
    delay(1500);
    touch2_val == 1;
    nh.spinOnce();
  }
  else if (touch1_val == 0 && touch2_val == 0){   //both touch
    drive(-1, 80, -1, 80);     //backward
    nh.loginfo("avoid_obstacle_door both touch");
    delay(1000);
    drive(1, 90, 1, 0);       //right
    delay(1500);
    drive(1, 90, 1, 80);     //forward
    delay(1500);
    
    touch1_val == 1;
    touch2_val == 1;
    nh.spinOnce();
  }
  else{
      nh.loginfo("avoid_obstacle_door move forward");
      drive(1, 100, 1, 95);    //forward
      nh.spinOnce();
  }
}


int searching_door(){
  door_num = calculate_ratio();
  if (door_num == 600 || door_num == 1500){
    drive(0, 0, 0, 0);
    return 1;
  }
  
  int starttime, endtime;
  starttime = millis();
  endtime = starttime;
  while ((endtime - starttime) <=4850)
  {
    drive(1, 0, 1, 125);                   //spin one round
    door_num = calculate_ratio();
    if (door_num == 600 || door_num == 1500){
      drive(0, 0, 0, 0);
      return 1;
    }
    nh.spinOnce();
    endtime = millis();
  }  
}
