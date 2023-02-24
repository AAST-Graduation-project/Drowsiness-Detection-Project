#!/usr/bin/env python3
import rospy
import cv2

from Include import EAR_Calculation_2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#creat bridge
bridge = CvBridge()
End = False
def my_callback(ros_frame):
    #start your code 
    rospy.loginfo("received a frame")

    global bridge
    #convert the frame to cv2
    cv2_frame = bridge.imgmsg_to_cv2(img_msg=ros_frame, desired_encoding="bgr8")

    #start calculations for EAR
    cv2_frame= EAR_Calculation_2.F_Get_Face(cv2_frame)
    rate.sleep()
    #showing the frame
    cv2.imshow("img", cv2_frame)
    #delay  
    cv2.waitKey(1) 
    #rate.sleep()

    # end your code 

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
    
        
        
