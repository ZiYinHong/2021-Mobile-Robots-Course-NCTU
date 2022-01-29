#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 
#include <stdio.h>
      
int flag = 0; 

// void callback(const std_msgs::Int32::ConstPtr& msg) {
//     printf("message from Arduino is %d\n", msg->data); 
//     flag = 1;
// }

int main(int argc, char **argv) 
{ 
 
    ros::init(argc, argv, "rpi_node");

    ros::NodeHandle node_obj;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::Publisher rpi_publisher_R = node_obj.advertise<std_msgs::Int32>("topic_rpisend_R", 10); 
    ros::Publisher rpi_publisher_L = node_obj.advertise<std_msgs::Int32>("topic_rpisend_L", 10);
    //ros::Subscriber rpi_subscriber = node_obj.subscribe("topic_arduinosend", 10, callback);
    //ros::Rate loop_rate(1); 

    
    while (ros::ok()) {
        std_msgs::Int32 R_pwm;
        std_msgs::Int32 L_pwm;
        
   
        std::cout << "user's right is :";
        std::cin >> R_pwm.data;
        std::cout << "user's left is :";
        std::cin >> L_pwm.data;
        flag = 1;
        
        if (flag == 1){
            rpi_publisher_R.publish(R_pwm);
            rpi_publisher_L.publish(L_pwm);
            flag == 0;
        }

        ros::spinOnce(); 
        //loop_rate.sleep();

    } 
    return 0;
}




