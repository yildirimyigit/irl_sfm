#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import Twist

prediction_velocities = np.load('/home/yigit/phd/yigit_phd_thesis/cnmp/output/1591828422/predicted_velocities.npy')

pub = rospy.Publisher('/pedbot/control/cmd_vel', Twist, queue_size=1)
rospy.init_node('prediction_runner', anonymous=True)

rate = rospy.Rate(14)
len_pred = np.shape(prediction_velocities)[0]
rospy.loginfo(len_pred)
vel = Twist()
for i in range(len_pred):
    vel.linear.x = prediction_velocities[i][0]
    vel.linear.y = prediction_velocities[i][1]
    rospy.loginfo(str(i)+': ' + str(prediction_velocities[i][0]) + '-' + str(prediction_velocities[i][1]))
    pub.publish(vel)
    rate.sleep()
    
vel = Twist()
pub.publish(vel)
