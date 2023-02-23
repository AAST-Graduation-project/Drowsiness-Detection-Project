#!/usr/bin/env python3
import rospy
import cv2


#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#creat bridge
global bridge
bridge = CvBridge()

def shutdown_callback():
    cap.release()
    rospy.loginfo("End of camera_node")
if __name__ == '__main__':
    
    # rospy.init_noed(node_name) --> start the node
    rospy.init_node("open_camera", anonymous=True)
    # open the camera
    cap= cv2.VideoCapture(0)
    # rate
    rate= rospy.Rate(20)
    # rospy.publisher(topic_name, msg_type, queue_size)
    pub = rospy.Publisher('my_topic', Image, queue_size=2)
    # call only on shutdown
    rospy.on_shutdown(shutdown_callback)
    while not rospy.is_shutdown():

        #get cv2 img
        ret,cv2_frame = cap.read()
        if ret is True:
            h, w, c= cv2_frame.shape
            resized = cv2.resize(cv2_frame, (int(w*0.6),int(h*0.6)))
            #convert cv2_img to ros img
            ros_frame = bridge.cv2_to_imgmsg(resized,"bgr8")

            #published to the topic
            rospy.loginfo("send a frame")
            pub.publish(ros_frame)
            

        #delay
        rate.sleep()
    




    
