#! /usr/bin/python

import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PoseHolder:
    def __init__(self):
        self.raf_pose = None
        self.raf_angle = None
        self.leo_pose = None
        self.leo_angle = None

pose_holder = PoseHolder()

rospy.init_node("task5")

def raf_callback(msg):
    pose_holder.raf_pose = (msg.x, msg.y)
    pose_holder.raf_angle = msg.theta
    
def leo_callback(msg):
    pose_holder.leo_pose = (msg.x, msg.y)
    pose_holder.leo_angle = msg.theta

rospy.Subscriber('/leo/pose', Pose, leo_callback)
rospy.Subscriber('/raf/pose', Pose, raf_callback)

pub_raf = rospy.Publisher('/raf/cmd_vel', Twist, queue_size=10)
        
def raf_move():
    if pose_holder.raf_pose is None or pose_holder.leo_pose is None:
        return
    pose_diff = np.array(pose_holder.leo_pose) - np.array(pose_holder.raf_pose)
    if np.linalg.norm(pose_diff) > 0:
        vel = float(rospy.get_param('/task5/speed'))
        pose_diff = pose_diff / np.linalg.norm(pose_diff)
        angle_delta = np.arctan2(*pose_diff[::-1]) - (pose_holder.raf_angle)
        while angle_delta > np.pi:
            angle_delta -= 2*np.pi
        while angle_delta < -np.pi:
            angle_delta += 2*np.pi
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = angle_delta
        pub_raf.publish(msg)

while not rospy.is_shutdown():
    raf_move()
    time.sleep(0.1)


