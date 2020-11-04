#! /usr/bin/python

import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

rospy.init_node("task3")
pub1 = rospy.Publisher('/raf/cmd_vel', Twist, queue_size=10)
pub2 = rospy.Publisher('/leo/cmd_vel', Twist, queue_size=10)

while True:
    msg = Twist()
    msg.linear.x = 2.0
    pub1.publish(msg)
    pub2.publish(msg)
    time.sleep(1)
    msg = Twist()
    msg.angular.z = np.pi / 2
    pub1.publish(msg)
    msg = Twist()
    msg.angular.z = -np.pi / 2
    pub2.publish(msg)
    time.sleep(1)


