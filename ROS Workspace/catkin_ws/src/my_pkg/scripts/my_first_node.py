#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

    


if __name__ == '__main__':

    #rospy.init_noed(node_name) --> start the node
    rospy.init_node("my_talker")

    #rospy.publisher(topic_name, msg_type, queue_size)
    pub = rospy.Publisher('my_topic', String, queue_size=5)

    #set rate to publish in Hz
    rate = rospy.Rate(10)

    #start your init

    #end your init

    #infinity_loop 
    while not rospy.is_shutdown():

        #start your loop code
        hello_str = "hello world"
        #end your loop code 

        #show on terminal
        rospy.loginfo(hello_str)

        #published to the topic
        pub.publish(hello_str)

        #delay
        rate.sleep()
    
   











#############
# import rospy
# import cv2
# if __name__=='__main__':
#     rospy.init_node("CAMERA_NODE")
#     rospy.loginfo("start_camera")
#     rate = rospy.Rate(10)
#     cap=cv2.VideoCapture(0)
    
   
#     while not rospy.is_shutdown():
#         ret,frame = cap.read()
#         cv2.imshow("my_frame",frame)
#         rate.sleep()
#         if cv2.waitKey(1)==ord("q"):
#             break
#     rospy.loginfo("END_camera")
