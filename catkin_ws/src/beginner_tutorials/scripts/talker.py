#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
# self-defined msg type
from beginner_tutorials.msg import my_msg

def talker():
	
    # publish topic 'chatter'
    # pub = rospy.Publisher('chatter', String, queue_size=10)  
    pub = rospy.Publisher('chatter', my_msg, queue_size=10)  # use self-defined msg
		
    # init node 'talker'
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    count = 1
    while not rospy.is_shutdown():

        # # fill-in message
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
				
        # # publish message
        # pub.publish(hello_str)
        #########################

        # use self-defined msg; fill-in message
        msg = my_msg()
        msg.id = countmsg.title = "hello"
        msg.content = "hello fromn python"
        # publish message
        pub.publish(msg)
        count = count + 1

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
