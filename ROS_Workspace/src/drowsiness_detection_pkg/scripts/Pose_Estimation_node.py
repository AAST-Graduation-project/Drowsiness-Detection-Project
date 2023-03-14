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
from Include import Pose_Estimation_Module
###########################################################
###########################################################

#creat bridge
bridge = CvBridge()

class static:
    DRIVER_POSE = "No Driver"
    COUNTER = 0

"""
name:   my_callback
Input:  ROS_Frame msg 
job:    Get the state of the driver by calculating Eye Aspect Ratio
output: cv2_Frame
"""
def my_callback(ros_frame):

    # Start your code for Pose_Estimation_node 
    rospy.loginfo("received a frame")

    #convert the frame to cv2
    global bridge
    

    # Start Pose Estimation (Half the Camera Freq.)
    static.COUNTER += 1
    if static.COUNTER == 4:
        cv2_frame = bridge.imgmsg_to_cv2(img_msg=ros_frame, desired_encoding="bgr8")
        # Get Driver's Pose'
        cv2_frame,static.DRIVER_POSE= Pose_Estimation_Module.F_Get_Pose(cv2_frame)

        # publish the Driver's Pose
        pub = rospy.Publisher("Pose_Result",String,queue_size=1)
        pub.publish(static.DRIVER_POSE)
        static.COUNTER = 0

    # End of code for Pose_Estimation_node     
   
   

"""
Function_name: shutdown_callback
Input: None
job: shutdown interrupt to close cv2_window & print end msg
output: None
"""
def shutdown_callback():
    rospy.loginfo("End of Pose_Estimation_Node")
    cv2.destroyAllWindows()


if __name__ == '__main__':

    # .init_noed(node_name) --> start the node
    rospy.init_node("Pose_Estimation")

    # set rate to publish in (x) Hz
    rate = rospy.Rate(4)

    # rospy.Subscriber(topic_name, Callback_Function, queue_size)
    sub = rospy.Subscriber('my_topic', Image, my_callback, queue_size=None)
    
    rospy.spin()
  
        
        
    # call only on shutdown
    rospy.on_shutdown(shutdown_callback)
    
        
        
