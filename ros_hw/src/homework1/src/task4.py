#! /usr/bin/python

import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def callback(msg):
    rospy.loginfo(msg)

rospy.init_node("task4")
rospy.Subscriber('/leo/pose', Pose, callback)
rospy.spin()

