#!/usr/bin/env python

import rospy

rospy.init_node('para', anonymous=False)

thing1 = rospy.get_param("~my_SN")
thing2 = rospy.get_param("~my_det")

print 'Serial', thing1, ' Detector: ', thing2
