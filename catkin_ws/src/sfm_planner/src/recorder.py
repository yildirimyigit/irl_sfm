#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

import numpy as np
import time


GOAL = np.array([rospy.get_param('/goal/position/x', 0), rospy.get_param('/goal/position/y', 14.0)])
OBS = np.array([rospy.get_param('/obstacle/position/x', 1), rospy.get_param('/obstacle/position/y', 0.0)]) # obstacle pose
SOURCE = np.array([0.0, -14.0])


class Recorder:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/robotPose", PoseStamped, self.pose_callback)
        self.vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.goal_reached_subscriber = rospy.Subscriber("/sfm/goal_reached", Bool, self.goal_reached_callback)
        data_path_prefix = '/home/yigit/Documents/projects/irl_sfm/data/demonstrations/sfm/1_obs_moving/novel/'
        # data_path_suffix = str(int(time.time()))
        data_path_suffix = str(OBS[1])
        self.data_path = data_path_prefix + data_path_suffix
        self.demonstration = []  # [[d_g_x, d_g_y, d_o_x, d_o_y, vx, vy], ...]
        self.last_vel_x, self.last_vel_y = 0, 0

    def vel_callback(self, msg):
        self.last_vel_x, self.last_vel_y = msg.linear.x, msg.linear.y
        
    def goal_reached_callback(self, msg):
        if msg.data: # is True
            self.save()
            rospy.signal_shutdown('Goal Reached!')

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
