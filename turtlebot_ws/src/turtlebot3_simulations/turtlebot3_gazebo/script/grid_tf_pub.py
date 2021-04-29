#!/usr/bin/env python3

import rospy
import tf
import numpy as np

class Localizer(object):
    """  This class may someday provide localization. 
    
    Currently it just broadcasts an identity transform from 'map' to
    'odom'.
    """
    
    def __init__(self):
        """ Initialize the localizer node. """
        
        rospy.init_node('grid_localizer')
        br = tf.TransformBroadcaster()
 
        # Broadcast the transform at 10 HZ
        while not rospy.is_shutdown():
            br.sendTransform((0., 0., 0.), (0., 0. , 0. , 1.),
                             rospy.Time.now(),
                             "odom",
                             "grid")
            rospy.sleep(.1)


if __name__ == '__main__':
    try:
        td = Localizer()
    except rospy.ROSInterruptException:
        pass