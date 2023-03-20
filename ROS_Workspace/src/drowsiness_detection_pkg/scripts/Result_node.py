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

###########################################################
###########################################################

# Creat Bridge
bridge = CvBridge()

# Global Variables
Eye_State = "No State"
Pose_State = "No State"

"""
Name:   EAR_callback
Input:  data String (Eye_state)
Job:    Set the State of Driver's EAR
Output: None
"""
def EAR_callback(msg_Eye_State):
    global Eye_State
    Eye_State = msg_Eye_State.data

"""
Name:   Pose_callback
Input:  data String (Pose_state)
Job:    Set the State of Driver's Pose 
Output: None
"""
def Pose_callback(msg_Pose_State):
    global Pose_State
    Pose_State = msg_Pose_State.data


"""
Name:   Camera_callback
Input:  ROS_Frame msg 
Job:    Get the Frame and Putting the Results on it
Output: None
"""
def Camera_callback(ros_frame):

    # Global Variables Call
    global Eye_State
    global Pose_State
    
    # Start your Code for Camera_node 
    rospy.loginfo("received a frame")

    # Convert the frame to cv2
    global bridge
    cv2_frame = bridge.imgmsg_to_cv2(img_msg=ros_frame, desired_encoding="bgr8")


    # Add the Results on Screen 
    # EAR
    cv2.putText(cv2_frame,"EAR_Result = "+str(Eye_State) , (10, 15),
                cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (200, 50, 50), 1)
    # Pose
    cv2.putText(cv2_frame,"Pose_Result = "+str(Pose_State) , (10, 35),
                cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (200, 50, 50), 1)
    
    #showing the frame
    cv2.imshow("Driver Camera", cv2_frame)

    #delay  
    cv2.waitKey(1)  # delay for showing frames 1 ms
    #rate.sleep()   # rate of receiving for the node

    # End your code for Camera_node

"""
Function_name: shutdown_callback
Input: None
job: shutdown interrupt to close cv2_window & print end msg
output: None
"""
def shutdown_callback():
    rospy.loginfo("End of Result_node")
    cv2.destroyAllWindows()


if __name__ == '__main__':

    # .init_noed(node_name) --> start the node
    rospy.init_node("Result")

    # set rate to publish in (x) Hz
    rate = rospy.Rate(10)

    # rospy.Subscriber(topic_name, msg_type, queue_size)
    # Subscriber for EAR 
    rospy.Subscriber('EAR_Result', String, EAR_callback, queue_size= None)

    # Subscriber for Pose Estimation
    rospy.Subscriber('Pose_Result', String, Pose_callback, queue_size= None)

    # Subscriber for Camera
    rospy.Subscriber('my_topic', Image, Camera_callback, queue_size= None)

    
    while (not rospy.is_shutdown()):
        # infinity_loop
        rospy.spin()
        
        
    # call only on shutdown
    rospy.on_shutdown(shutdown_callback)
    
        
        
