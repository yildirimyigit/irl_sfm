#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

import numpy as np
import time


GOAL = np.array([rospy.get_param('/goal/position/x', 0), rospy.get_param('/goal/position/y', 13.0)])
# OBS = np.array([rospy.get_param('/obstacle/position/x', 1), rospy.get_param('/obstacle/position/y', 0.0)]) # obstacle pose
SOURCE = np.array([rospy.get_param('/start/position/x', 0), rospy.get_param('/start/position/y', 0.0)])


class Recorder:
    '''
        @param obs_pose = np.array([x, y])
    '''
    def __init__(self, in_data_path='/home/yigit/Documents/projects/irl_sfm/data/demonstrations/sfm/'):
        self.demonstration = []  # [[d_g_x, d_g_y, d_o_x, d_o_y, vx, vy, x, y], ...]
        self.last_vel_x, self.last_vel_y = 0, 0

        self.is_saved = False
        self.obstacle_pose_read = False
        self.obstacle_pose = PoseStamped()
        self.OBS = np.zeros(2)

        self.pose_subscriber = rospy.Subscriber("/robotPose", PoseStamped, self.pose_callback)
        self.vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.obs_0_pose_subs = rospy.Subscriber("/obstacle_0_pose", PoseStamped, self.obs_0_pose_callback)
        self.goal_reached_subscriber = rospy.Subscriber("/sfm/goal_reached", Bool, self.goal_reached_callback)

        self.data_path_root = rospy.get_param('/recorder_root_path', in_data_path)

    def vel_callback(self, msg):
        self.last_vel_x, self.last_vel_y = msg.linear.x, msg.linear.y

    def obs_0_pose_callback(self, msg):
        if not self.obstacle_pose_read:  # run once
            self.obstacle_pose = msg
            data_path_prefix = 'sx_' + str(SOURCE[0]) + '_sy_' + str(SOURCE[1]) + '_gx_' + str(GOAL[0]) + '_gy_' + str(GOAL[1])
            self.OBS = np.array([self.obstacle_pose.pose.position.x, self.obstacle_pose.pose.position.y])
            data_path_suffix = 'x_' + str(round(self.OBS[0], 1)) + '_y_' + str(round(self.OBS[1], 1))
            self.data_path = self.data_path_root + data_path_prefix + '_' + data_path_suffix

            self.obstacle_pose_read = True

    def goal_reached_callback(self, msg):
        if msg.data:  # is True
            self.save()
            self.is_saved = True
            rospy.signal_shutdown('Goal Reached!')

    def pose_callback(self, msg):
        if self.obstacle_pose_read:
            if msg.pose.position.y >= SOURCE[1]:
                pose_x = msg.pose.position.x
                pose_y = msg.pose.position.y
                pose_arr = np.array([pose_x, pose_y])
                d_g_x, d_g_y = GOAL - pose_arr
                d_o_x, d_o_y = self.OBS - pose_arr

                vx, vy = self.last_vel_x, self.last_vel_y

                state = np.array([d_g_x, d_g_y, d_o_x, d_o_y, vx, vy, pose_x, pose_y])
                self.demonstration.append(state)

    def save(self):
        np.save(self.data_path, np.array(self.demonstration))


if __name__ == '__main__':
    rospy.init_node('sfm_data_recorder', anonymous=True)
    r = Recorder('/home/yigit/Documents/projects/irl_sfm/data/demonstrations/sfm/small_env_changing_s_g/')
    rospy.spin()
