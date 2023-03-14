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


#creat bridge
global bridge
bridge = CvBridge()

"""
Function_name: shutdown_callback
Input: None
job: shutdown interrupt to close the camera & print end msg
output: None
"""
def shutdown_callback():
    cap.release()
    rospy.loginfo("End of camera_node")

"""
Function_name: main
Input: None
job: contain camera publisher code
output: None
"""
if __name__ == '__main__':
    
    # rospy.init_noed(node_name) --> start the node
    rospy.init_node("open_camera", anonymous=True)

    # open the camera
    cap= cv2.VideoCapture(0)

    # rate
    rate= rospy.Rate(10)

    # rospy.publisher(topic_name, msg_type, queue_size)
    pub = rospy.Publisher('my_topic', Image, queue_size=1)
  

    # call only on shutdown
    rospy.on_shutdown(shutdown_callback)

    while not rospy.is_shutdown():

        # get cv2 img
        ret,cv2_frame = cap.read()

        if ret is True:
            # crop the image to improve the speed
            resized = cv2.resize(cv2_frame,None,fx=0.7,fy=0.7)

            # convert cv2_img to ros img
            ros_frame = bridge.cv2_to_imgmsg(resized,"bgr8")

            # published to the topic
            rospy.loginfo("send a frame")
            pub.publish(ros_frame)
            rate.sleep()

        else:
            # print error msg
            rospy.loginfo_once(msg="cannot open the camera")    
            

        #delay
        
    




    
