#!/usr/bin/env python
# run on raspberry pi (python版) 可work

import rospy
 
from std_msgs.msg import Int32

#global rec_flag
 
return_msg=None
 
#define the display text
def number_callback(msg):
    global rec_flag
    print "message from Arduino is : {}" .format(msg) 
    rec_flag = True
 
 
if __name__=='__main__':
    rospy.init_node('rpi_node')
    rospy.Subscriber("topic_arduinosend",Int32, number_callback)
    rpi_publisher = rospy.Publisher("topic_rpisend", Int32, queue_size=10)
    rate=rospy.Rate(1)

    global rec_flag
    rec_flag = True

    while not rospy.is_shutdown():
        if rec_flag == True:
            msg = input("user's input is: ")
            rpi_publisher.publish(msg)
            rec_flag = False
        
     
 
   
