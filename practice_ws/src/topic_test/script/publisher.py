#!/usr/bin/env python3

import rospy


rospy.init_node('python_pub_node')

while not rospy.is_shutdown():

    rospy.loginfo('Hello')
    rospy.sleep(1)
