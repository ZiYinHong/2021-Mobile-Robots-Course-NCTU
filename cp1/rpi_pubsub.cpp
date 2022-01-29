//run on raspberry pi (c++版本) 可work
#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 
#include <stdio.h>
      
int flag = 1; 

void number_callback(const std_msgs::Int32::ConstPtr& msg) {
    printf("message from Arduino is %d\n", msg->data); 
    flag = 1;
}

int main(int argc, char **argv) 
{ 
 
    ros::init(argc, argv, "rpi_node");

    ros::NodeHandle node_obj;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::Publisher rpi_publisher = node_obj.advertise<std_msgs::Int32>("topic_rpisend", 10); 
    ros::Subscriber rpi_subscriber = node_obj.subscribe("topic_arduinosend", 10, number_callback);
    ros::Rate loop_rate(1); 

    
    while (ros::ok()) {
        std_msgs::Int32 msg;
        
        if (flag == 1){
            std::cout << "user's input is :";
            std::cin >> msg.data;
            rpi_publisher.publish(msg);
            flag = 0;
        }
               
        
        ros::spinOnce(); 
        loop_rate.sleep();

    } 
    return 0;
}




