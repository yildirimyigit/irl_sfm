#!/usr/bin/env python3.7

from keras.models import Model, load_model
import keras.losses
import tensorflow as tf
import tensorflow_probability as tfp

import numpy as np

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3, PoseStamped, Point, Vector3Stamped, Quaternion
from std_msgs.msg import Header, Float32MultiArray


def custom_loss(y_true, y_predicted):
    mean, log_sigma = tf.split(y_predicted, 2, axis=-1)
    y_true_value, temp = tf.split(y_true, 2, axis=-1)
    sigma = tf.nn.softplus(log_sigma)
    dist = tfp.distributions.MultivariateNormalDiag(loc=mean, scale_diag=sigma)
    loss = -tf.reduce_mean(dist.log_prob(y_true_value))
    return loss


class OnlineCNMPRunner():
    def __init__(self):
        self.node_name = "/sim0"
        model_path = f'/home/yigit/phd/yigit_phd_thesis/cnmp/output/sfm/small_env_changing_s_g/1607473236/cnmp_best_validation.h5'
        keras.losses.custom_loss = custom_loss
        self.model = load_model(f'{model_path}', custom_objects={'tf': tf})
        self.latent_layer = Model(inputs=self.model.input, outputs=self.model.get_layer('obs_mlp').get_output_at(-1))

        self.d_x, self.d_y, self.d_gamma = 2, 2, 2
        self.latent_shape = (1, 1, 128)

        #####################
        self.pose_subs = rospy.Subscriber(self.node_name + "/robotPose", PoseStamped, self.pose_callback)
        
        self.nof_obstacles = rospy.get_param('/nof_obstacles', 10)
        self.obstacles_pose_subs = []
        self.obstacle_poses = []
        
        for i in range(self.nof_obstacles):
            self.obstacles_pose_subs.append(rospy.Subscriber(self.node_name + "/obstacle_" + str(i) + "_pose", PoseStamped, self.obs_pose_callback, (i)))
            self.obstacle_poses.append(PoseStamped())

        self.pub = rospy.Publisher(self.node_name + '/cmd_vel', Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/state_sub', Float32MultiArray, queue_size=1)
        #####################

        self.goal_pose = PoseStamped(Header(0, 0, 'odom'), Pose(Point(rospy.get_param('/goal/position/x', 0.0), rospy.get_param('/goal/position/y', 13.0), 0), Quaternion(0, 0, rospy.get_param('/goal/orientation/z', 0.706),rospy.get_param('/goal/orientation/w', 0.707))))

        self.last_pose = PoseStamped()
        
        self.observations = np.zeros((self.nof_obstacles, 1, self.d_x+self.d_gamma+self.d_y))

    def predict_model(self, observation, target_X):  # observation and target_X contain gamma values too
        predicted_Y = np.zeros((1, self.d_y))
        predicted_std = np.zeros((1, self.d_y))
        prediction = self.model.predict([observation, target_X])[0]
        predicted_Y = prediction[:, :self.d_y]
        predicted_std = np.log(1+np.exp(prediction[:, self.d_y:]))
        
        return predicted_Y, predicted_std

    def pose_callback(self, msg):
        self.last_pose = msg
    
    def obs_pose_callback(self, msg, index):
        self.obstacle_poses[index] = msg
        # conditioned on the observation
        self.observations[index] = np.array([0, 0, self.obstacle_poses[index].pose.position.x, self.obstacle_poses[index].pose.position.y - self.goal_pose.pose.position.y, 0.0, 0.0]).reshape(1, 1, self.d_x+self.d_gamma+self.d_y)
        
    def calculate_distance(self, p0, p1):
        distance = Vector3()
        distance.x = p1.pose.position.x - p0.pose.position.x
        distance.y = p1.pose.position.y - p0.pose.position.y
        return distance
        
    def calculate_euclidean_distance(self, p0, p1):
        distance = ((p1.pose.position.x - p0.pose.position.x)**2 + (p1.pose.position.y - p0.pose.position.y)**2)**0.5
        return distance

    def distance_to_closest_obstacle(self, pose, obstacles):  # obstacles in the front
        min_distance = 1000000
        min_distance_id = -1
        for i, p in obstacles.items():
            dist = self.calculate_euclidean_distance(pose, p)
            # rospy.loginfo(str(i)+' '+str(dist))
            if dist < min_distance:
                min_distance = dist
                min_distance_id = i
                
        return min_distance_id, min_distance

    def execute(self):
        rate = rospy.Rate(10)
        
        last_passed = -1
        while not rospy.is_shutdown():
            vel = Twist()
            
            distance_to_goal = self.calculate_distance(self.last_pose, self.goal_pose)  # X
            
            obstacles_in_front = {}
            for i in range(self.nof_obstacles-1, -1, -1):
                if self.last_pose.pose.position.y <= self.obstacle_poses[i].pose.position.y:  # obstacle is in the front
                    obstacles_in_front.update({i:self.obstacle_poses[i]})

            if len(obstacles_in_front) == 0:  # if the last one is passed, use the closest as the gamma and observation
                obstacles_in_front.update({last_passed:self.obstacle_poses[last_passed]})

            obstacle, dist = self.distance_to_closest_obstacle(self.last_pose, obstacles_in_front)

            distance_to_obs = self.calculate_distance(self.last_pose, self.obstacle_poses[obstacle])  # Gamma
            observation = self.observations[obstacle].reshape(1, 1, self.d_x+self.d_gamma+self.d_y)
                
            target_X_Gamma = np.array([distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y]).reshape(1, 1, self.d_x+self.d_gamma)
            
            # rospy.loginfo('***')
            predicted_Y, predicted_std = self.predict_model(observation, target_X_Gamma)
            
            # ll_prediction = self.latent_layer.predict([observation, target_X_Gamma]).reshape(self.latent_shape[2])
            # state = ll_prediction.tolist()
            
            # 6D state-action
            # state = [distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y, predicted_Y[0][0], predicted_Y[0][1]]
            # 4D state
            # state = [distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y]
            # <self.latent_shape[2]>D latent encoding
            # 2D state
            state = [distance_to_obs.x, distance_to_obs.y]
            
            state_array = Float32MultiArray(data=state)
            self.state_pub.publish(state_array)
            
            if obstacle != -1:
                last_passed = obstacle
                vel.linear.x = predicted_Y[0][0]
                vel.linear.y = predicted_Y[0][1]
                self.pub.publish(vel)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('online_sfm_control_with_cnmp')
    ocr = OnlineCNMPRunner()
    ocr.execute()
