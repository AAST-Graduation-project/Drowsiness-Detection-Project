#!/usr/bin/env python3
import rospy
import cv2
from Include import EAR_Calculation

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#creat bridge
bridge = CvBridge()

def my_callback(ros_frame):
    #start your code 
    global bridge
    cv2_frame = bridge.imgmsg_to_cv2(ros_frame,"bgr8")
    #frame = cv2.resize(cv2_frame, width=400)
    cv2_frame= EAR_Calculation.Get_Face(cv2_frame)
    cv2.imshow("img", cv2_frame)
    cv2.waitKey(1)
    #delay
    #rate.sleep()
    # end your code   

if __name__ == '__main__':

    #rospy.init_noed(node_name) --> start the node
    rospy.init_node("EAR")

    #set rate to publish in (x) Hz
    rate = rospy.Rate(30)

    #rospy.Subscriber(topic_name, msg_type, queue_size)
    rospy.Subscriber('my_topic', Image, my_callback, queue_size= 1)
    #infinity_loop
    rospy.spin()