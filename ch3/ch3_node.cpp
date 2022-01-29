#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 
#include <stdio.h>
#include <wiringPi.h>
      
int flag = 0; 

int main(int argc, char **argv) 
{ 
 
    ros::init(argc, argv, "ch3_node");

    ros::NodeHandle node_obj;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::Publisher rpi_publisher_start = node_obj.advertise<std_msgs::Int32>("topic_rpisend_start", 10); 
    

    while (ros::ok()) {
        std_msgs::Int32 start;
         
        std::cout << "when start give 1 :";
        std::cin >> start.data;
        flag = 1;
        
        if (flag == 1){
            rpi_publisher_start.publish(start);
            flag == 0;
        }

        ros::spinOnce(); 
    } 

    return 0;
}




