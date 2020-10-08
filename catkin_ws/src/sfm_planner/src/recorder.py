#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist

import numpy as np
import time


GOAL = np.array([rospy.get_param('/goal/position/x', 2.2), rospy.get_param('/goal/position/y', -6.0)])
OBS = np.array([1.0, -10.0]) # obstacle pose
SOURCE = np.array([0.0, -14.0])


class Recorder:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/robotPose", PoseStamped, self.pose_callback)
        self.vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        data_path_prefix = '/home/yigit/Documents/projects/irl_sfm/data/demonstrations/sfm/1_obs_multi_goals/'
        # data_path_suffix = str(int(time.time()))
        data_path_suffix = str(GOAL[0])
        self.data_path = data_path_prefix + data_path_suffix
        self.demonstration = []  # [[d_g_x, d_g_y, d_o_x, d_o_y, vx, vy], ...]
        self.last_vel_x, self.last_vel_y = 0, 0

    def vel_callback(self, msg):
        self.last_vel_x, self.last_vel_y = msg.linear.x, msg.linear.y

    def pose_callback(self, msg):
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        pose_arr = np.array([pose_x, pose_y])
        d_g_x, d_g_y = GOAL - pose_arr
        d_o_x, d_o_y = OBS - pose_arr
        
        vx, vy = self.last_vel_x, self.last_vel_y
        
        state = np.array([d_g_x, d_g_y, d_o_x, d_o_y, vx, vy])
        self.demonstration.append(state)

    def save(self):
        np.save(self.data_path, np.array(self.demonstration))


if __name__ == '__main__':
    rospy.init_node('sfm_data_recorder', anonymous=True)
    r = Recorder()
    rospy.spin()
    r.save()
