//run on Arduino
#include <ros.h> 
#include <std_msgs/Int32.h>

std_msgs::Int32 msg;
std_msgs::Int32 return_msg;
ros::NodeHandle nh;


ros::Publisher arduino_pub("topic_arduinosend", &return_msg);


void messageCb(const std_msgs::Int32 &msg){
  return_msg.data = msg.data * 2;
  arduino_pub.publish(&return_msg); 
}

ros::Subscriber<std_msgs::Int32> arduino_sub("topic_rpisend", messageCb);


void setup() {
  //Serial.begin(57600);
  nh.initNode(); 
  nh.advertise(arduino_pub);
  nh.subscribe(arduino_sub);
}


void loop() {
  //Serial.print(msg.data, return_msg.data);
  //ROS_INFO("arduino get [%d]", msg.data);
  //ROS_INFO("return_msg.data  [%d]", return_msg.data);

  nh.spinOnce(); 
}
