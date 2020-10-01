#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry

poses = []

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    poses.append(np.array([x, y]))
    

def listener():
    rospy.init_node('pose_recorder', anonymous=True)

    rospy.Subscriber('/pedsim_simulator/robot_position', Odometry, callback)
    rospy.spin()
    

def save():
    np.save('/home/yigit/phd/yigit_phd_thesis/cnmp/output/1591828422/poses.npy', np.array(poses))


if __name__ == '__main__':
    listener()
    save()

