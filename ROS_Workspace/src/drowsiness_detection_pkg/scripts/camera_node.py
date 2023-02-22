#!/usr/bin/env python3
import rospy
import cv2


#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#creat bridge
global bridge
bridge = CvBridge()


if __name__ == '__main__':
    
    #rospy.init_noed(node_name) --> start the node
    rospy.init_node("open_camera", anonymous=True)
    #open the camera
    cap= cv2.VideoCapture(0)
    #rate
    rate= rospy.Rate(10)
    #rospy.publisher(topic_name, msg_type, queue_size)
    pub = rospy.Publisher('my_topic', Image, queue_size=1)
   
    while not rospy.is_shutdown():

        #get cv2 img
        ret,cv2_frame = cap.read()
        if ret is True:
            
            resized = cv2.resize(cv2_frame, (480,480), interpolation = cv2.INTER_AREA)
            #convert cv2_img to ros img
            ros_frame = bridge.cv2_to_imgmsg(resized,"bgr8")

            #published to the topic
            rospy.loginfo("send a frame")
            pub.publish(ros_frame)

        if cv2.waitKey(1) == ord("q") or rospy.is_shutdown():

            cap.release()
            break

        #delay
        rate.sleep()



    
