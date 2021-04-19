#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def listener():

    # init node 'listener'
    rospy.init_node('listenercpp', anonymous=True)

    # subscribe to topic 'chatter'
    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
