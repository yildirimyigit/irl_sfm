#!/usr/bin/env python3.7

from keras.models import Model, load_model
import keras.losses
import tensorflow as tf
import tensorflow_probability as tfp

import numpy as np
import time
import os

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3, PoseStamped, Point, Vector3Stamped, Quaternion
from std_msgs.msg import Header


def custom_loss(y_true, y_predicted):
    mean, log_sigma = tf.split(y_predicted, 2, axis=-1)
    y_true_value, temp = tf.split(y_true, 2, axis=-1)
    sigma = tf.nn.softplus(log_sigma)
    dist = tfp.distributions.MultivariateNormalDiag(loc=mean, scale_diag=sigma)
    loss = -tf.reduce_mean(dist.log_prob(y_true_value))
    return loss


class OnlineCNMPRunner:
    def __init__(self):
        self.root_path = f'/home/yigit/phd/yigit_phd_thesis/cnmp/output/sfm/small_env_changing_s_g/1607473236/'
        model_path = f'{self.root_path}cnmp_best_validation.h5'
        keras.losses.custom_loss = custom_loss
        self.model = load_model(f'{model_path}', custom_objects={'tf': tf})

        self.d_x, self.d_y, self.d_gamma = 2, 2, 2

        #####################
        self.pose_subs = rospy.Subscriber("/robotPose", PoseStamped, self.pose_callback)
        self.obs_0_pose_subs = rospy.Subscriber("/obstacle_0_pose", PoseStamped, self.obs_0_pose_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #####################

        self.goal_pose = PoseStamped(Header(0, 0, 'odom'), Pose(Point(rospy.get_param('/goal/position/x', 0.0), rospy.get_param('/goal/position/y', 13.0), 0), Quaternion(0, 0, rospy.get_param('/goal/orientation/z', 0.706),rospy.get_param('/goal/orientation/w', 0.707))))

        self.last_pose = PoseStamped()
        self.obstacle_pose = PoseStamped()
        #self.obstacle_pose.pose.position.x = 1
        #self.obstacle_pose.pose.position.y = -10
        
        self.observation = np.zeros((1, 1, self.d_x+self.d_gamma+self.d_y))
        
        ###
        
        self.states = []  # [[d_g_x, d_g_y, d_o_x, d_o_y, vx, vy], ...]


    def predict_model(self, observation, target_X):  # observation and target_X contain gamma values too
        predicted_Y = np.zeros((1, self.d_y))
        predicted_std = np.zeros((1, self.d_y))
        prediction = self.model.predict([observation, target_X])[0]
        predicted_Y = prediction[:, :self.d_y]
        predicted_std = np.log(1+np.exp(prediction[:, self.d_y:]))
        
        return predicted_Y, predicted_std

    def pose_callback(self, msg):
        self.last_pose = msg
    
    def obs_0_pose_callback(self, msg):
        self.obstacle_pose = msg
        self.observation = np.array([0, 0, self.obstacle_pose.pose.position.x-self.goal_pose.pose.position.x, self.goal_pose.pose.position.y-self.obstacle_pose.pose.position.y, 0.0, 0.0]).reshape(1, 1, self.d_x+self.d_gamma+self.d_y)

    def calculate_distance(self, p0, p1):
        distance = Vector3()
        distance.x = p1.pose.position.x - p0.pose.position.x
        distance.y = p1.pose.position.y - p0.pose.position.y
        return distance

    def execute(self):
        rate = rospy.Rate(3)
        
        #observation = np.array([0.0831583e-02, 2.8e+01, 1.006e+00, 14.0, 0.0, 2.01129232e-04]).reshape(1, 1, self.d_x+self.d_gamma+self.d_y)
        
        while not rospy.is_shutdown():
            vel = Twist()
            
            distance_to_goal = self.calculate_distance(self.last_pose, self.goal_pose)  # X
            distance_to_obs = self.calculate_distance(self.last_pose, self.obstacle_pose)  # Gamma
            
            target_X_Gamma = np.array([distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y]).reshape(1, 1, self.d_x+self.d_gamma)
            rospy.loginfo(target_X_Gamma)
            #rospy.loginfo(self.observation)
            predicted_Y, predicted_std = self.predict_model(self.observation, target_X_Gamma)
            
            vel.linear.x = predicted_Y[0][0]
            vel.linear.y = predicted_Y[0][1]
            self.pub.publish(vel)
            self.states.append(np.array([distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y, predicted_Y[0][0], predicted_Y[0][1]]))
            rate.sleep()

    def save_states(self):
        runs_path = f'{self.root_path}/runs/'
        try:
            os.mkdir(runs_path)
        except:
            pass
        path_suffix = 'x_' + str(round(self.obstacle_pose.pose.position.x, 1)) + '_y_' + str(round(self.obstacle_pose.pose.position.y, 1))
        this_run_path = runs_path + path_suffix + '_' + str(int(time.time())) + '.npy'
        np.save(this_run_path, np.array(self.states))
    

if __name__ == '__main__':
    rospy.init_node('online_sfm_control_with_cnmp')
    ocr = OnlineCNMPRunner()
    ocr.execute()
    ocr.save_states()
