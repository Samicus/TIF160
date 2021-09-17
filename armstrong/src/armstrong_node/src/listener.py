#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    print(data.data)

if __name__ == '__main__':

    rospy.init_node('listener')
    rate = rospy.Rate(1)  # Adjust rate?

    while not rospy.is_shutdown():
        rospy.Subscriber('chatter', String, callback)
        rate.sleep()
