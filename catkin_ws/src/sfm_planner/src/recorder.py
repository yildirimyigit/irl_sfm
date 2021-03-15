#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

import numpy as np
import time
import sys



class Recorder:
  def __init__(self):
  
    self.GOAL = np.array([rospy.get_param('goal/position/x', 0), rospy.get_param('goal/position/y', 13.0)])
    self.OBS = np.array([rospy.get_param('obstacle/position/x', 0.0), rospy.get_param('obstacle/position/y', 0.0)]) # obstacle pose
    self.SOURCE = np.array([rospy.get_param('start/position/x', 0.0), rospy.get_param('start/position/y', 0.0)])
  
    self.demonstration = []  # [[d_g_x, d_g_y, d_o_x, d_o_y, vx, vy, x, y], ...]
    self.last_vel_x, self.last_vel_y = 0, 0

    self.is_saved = False
    
    self.data_path_root = rospy.get_param('/recorder_root_path')

    self.pose_subscriber = rospy.Subscriber("robotPose", PoseStamped, self.pose_callback)
    self.vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
    self.goal_reached_subscriber = rospy.Subscriber("sfm/goal_reached", Bool, self.goal_reached_callback)

  def vel_callback(self, msg):
    self.last_vel_x, self.last_vel_y = msg.linear.x, msg.linear.y

  def goal_reached_callback(self, msg):
    if msg.data:  # is True
      self.save()
      self.is_saved = True
      rospy.signal_shutdown('Goal Reached!')

  def pose_callback(self, msg):
    if msg.pose.position.y >= self.SOURCE[1]:
      pose_x = msg.pose.position.x
      pose_y = msg.pose.position.y
      pose_arr = np.array([pose_x, pose_y])
      d_g_x, d_g_y = self.GOAL - pose_arr
      d_o_x, d_o_y = self.OBS - pose_arr

      vx, vy = self.last_vel_x, self.last_vel_y

      state = np.array([d_g_x, d_g_y, d_o_x, d_o_y, vx, vy, pose_x, pose_y])
      self.demonstration.append(state)

  def save(self):
    filename = str(self.SOURCE[0]) + '_' + str(self.GOAL[0]) + '_' + str(self.OBS[0]) + '_' + str(self.OBS[1]) + '.npy'
    save_path = self.data_path_root + rospy.get_param('/scenario') + '/' + filename

    np.save(save_path, np.array(self.demonstration))


if __name__ == '__main__':
  rospy.init_node('sfm_data_recorder', anonymous=True)
  
  r = Recorder()
  rospy.spin()

