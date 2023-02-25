#!/usr/bin/env python3
###########################################################
###########################################################
# ROS Dependecies
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Import Libraries
import cv2

# Import Files
from Include import EAR_Calculation_2
###########################################################
###########################################################

#creat bridge
bridge = CvBridge()

"""
name:   my_callback
Input:  ROS_Frame msg 
job:    Get the state of the driver by calculating Eye Aspect Ratio
output: cv2_Frame
"""
def my_callback(ros_frame):

    #start your code for EAR_node 
    rospy.loginfo("received a frame")

    #convert the frame to cv2
    global bridge
    cv2_frame = bridge.imgmsg_to_cv2(img_msg=ros_frame, desired_encoding="bgr8")

    #start calculations for EAR
    cv2_frame= EAR_Calculation_2.F_Get_Face(cv2_frame)

    
    #showing the frame
    cv2.imshow("img", cv2_frame)

    #delay  
    cv2.waitKey(1)  # delay for showing frames 1 ms
    rate.sleep()   # rate of receiving for the node

    # end your code for EAR_node

"""
Function_name: shutdown_callback
Input: None
job: shutdown interrupt to close cv2_window & print end msg
output: None
"""
def shutdown_callback():
    rospy.loginfo("End of EAR_node")
    cv2.destroyAllWindows()


if __name__ == '__main__':

    # .init_noed(node_name) --> start the node
    rospy.init_node("EAR")

    # set rate to publish in (x) Hz
    rate = rospy.Rate(20)

    # rospy.Subscriber(topic_name, msg_type, queue_size)
    rospy.Subscriber('my_topic', Image, my_callback, queue_size= 1)
    
    while (not rospy.is_shutdown()):
        # infinity_loop
        rospy.spin()
        
        
    # call only on shutdown
    rospy.on_shutdown(shutdown_callback)
    
        
        
