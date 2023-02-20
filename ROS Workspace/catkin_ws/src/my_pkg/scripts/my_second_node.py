#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def my_callback(my_string):
    #start your code 
    #data is in msg.String
    rospy.loginfo("i heard "+my_string.data)
    #delay
    rate.sleep()
    # end your code    


if __name__ == '__main__':

    #rospy.init_noed(node_name) --> start the node
    rospy.init_node("my_listener")

    #set rate to publish in (x) Hz
    rate = rospy.Rate(1)

    #rospy.Subscriber(topic_name, msg_type, queue_size)
    rospy.Subscriber('my_topic', String, my_callback, queue_size= 5)

    #start your init

    #end your init
    
    #infinity_loop
    rospy.spin()
    
