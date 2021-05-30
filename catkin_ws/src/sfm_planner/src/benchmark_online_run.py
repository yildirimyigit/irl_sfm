#!/usr/bin/env python3.7

import torch
import torch.nn as nn
import torch.nn.functional as F

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose, Quaternion, Vector3, Point
from std_msgs.msg import Header, Float32MultiArray


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(4, 300)
        self.fc2 = nn.Linear(300, 400)
        self.fc3 = nn.Linear(400, 2)

    def forward(self, x):  # x are the states: [d_obs_x, d_obs_y, d_goal_x, d_goal_y]
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class BenchmarkRunner:
    def __init__(self):
        model_path = f'/home/yigit/Documents/projects/irl_sfm/python_ws/nn_baseline_for_cnmp/model/model_state_dict.pt'
        self.model = Net()
        self.model.load_state_dict(torch.load(model_path))

        self.d_x, self.d_y, self.d_gamma = 2, 2, 2

        self.pose_sub = rospy.Subscriber("/sim0/robotPose", PoseStamped, self.pose_callback)
        self.obs_0_pose_sub = rospy.Subscriber("/sim0/obstacle_0_pose", PoseStamped, self.obs_0_pose_callback)
        self.pub = rospy.Publisher('/sim0/cmd_vel', Twist, queue_size=1)
        # self.state_pub = rospy.Publisher('/sim0/state_sub', Float32MultiArray, queue_size=1)

        self.goal_pose = PoseStamped(Header(0, 0, 'odom'), Pose(Point(rospy.get_param('/goal/position/x', 2.0), rospy.get_param('/goal/position/y', 13.0), 0), Quaternion(0, 0, rospy.get_param('/goal/orientation/z', 0.706),rospy.get_param('/goal/orientation/w', 0.707))))

        self.last_pose = PoseStamped()
        self.obstacle_pose = PoseStamped()
        self.state = torch.zeros((self.d_x+self.d_gamma))

    def pose_callback(self, msg):
        self.last_pose = msg

    def obs_0_pose_callback(self, msg):
        self.obstacle_pose = msg
        
    def calculate_distance(self, p0, p1):
        distance = Vector3()
        distance.x = p1.pose.position.x - p0.pose.position.x
        distance.y = p1.pose.position.y - p0.pose.position.y
        return distance

    def execute(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            vel = Twist()

            distance_to_goal = self.calculate_distance(self.last_pose, self.goal_pose)  # X
            distance_to_obs = self.calculate_distance(self.last_pose, self.obstacle_pose)  # Gamma
            state = torch.Tensor([distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y]).view(1, self.d_x+self.d_gamma)
            rospy.loginfo(state)
            
            predicted_Y = self.model(state)

            vel.linear.x = predicted_Y[0][0]
            vel.linear.y = predicted_Y[0][1]
            self.pub.publish(vel)

            # state = [distance_to_goal.x, distance_to_goal.y, distance_to_obs.x, distance_to_obs.y, predicted_Y[0][0], predicted_Y[0][1]]
            # self.states.append(np.array(state))
            # state_array = Float32MultiArray(data=state)
            # self.state_pub.publish(state_array)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('benchmark_nn_online_test')
    br = BenchmarkRunner()
    br.execute()
    # br.save_states()
