#include <ros.h>
#include <stdlib.h> 
#include <std_msgs/Int32.h>

#define IR_PIN 2

int start_val;
int starttime,  endtime;
int zero_count, one_count, total_count;
float ratio;
int IR_value;
char INFO[20], INFO1[20];

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
   pinMode(IR_PIN,INPUT);

  
}


void loop()
{  
  nh.spinOnce();
  

  
  if (start_val == 1){
    nh.loginfo("start");
//    starttime = millis();
//    endtime = starttime;
//    while ((endtime - starttime) <=100) // do this loop for up to 100mS
//    {
//      IR_value = digitalRead(IR_PIN); 
//      sprintf(INFO, "IR_value : %d", IR_value);
//      //nh.loginfo(INFO);
//      
//      if (IR_value == 1){
//         one_count++;
//      }
//      else{
//        zero_count++;
//      }
//      delay(1);
//      endtime = millis();
//    } 
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
    
    if (ratio > 0.2 && ratio < 0.3){
       nh.loginfo("goal door 1500");
    }
    else if (ratio > 0.4 && ratio < 0.5){
      nh.loginfo("goal door 600");
    }
    
    one_count = 0; 
    zero_count = 0;
  }
  
  delay(500);

}
