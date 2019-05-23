import rosbag
import sys
import math
from environment import Environment
from environment import Point
from environment import Action
from environment import State

# README::
# This program gets .bag file name and topic name as argument:
# USAGE: python get_data.py <bag_file>

# These arrays blow hold the data as 2D arrays. 1st dimension represents which message
# the data comes from
# 2nd dimension represents which person is this
# Ex: pose_positions_x[i][j] means x position of jth person at ith message (iteration)
pose_positions_x = []
pose_positions_y = []
pose_orientations_z = []
pose_orientations_w = []

twist_linears_x = []
twist_linears_y = []
twist_angulars_z = []

# This array will be filled with distance of two agents in each iteration
distances = []


def calculate_distances():
	for i in range(len(pose_positions_x)):
		x_distance = (pose_positions_x[i][0] - pose_positions_x[i][1]) ** 2
		y_distance = (pose_positions_y[i][0] - pose_positions_y[i][1]) ** 2
		distance = (x_distance + y_distance) ** (1.0/2.0)
		distances.append(distance)


def initialize_positions(bag):
	print('+ initialize_positions()')
	messages = []
	topics = []
	times = []
	tracks = []
	for topic, msg, t in bag.read_messages():
		messages.append(msg)
		topics.append(topic)
		times.append(t)
		tracks.append(msg.tracks)

	for i in range(len(tracks)):
		pose_positions_x.append([])
		pose_positions_y.append([])
		pose_orientations_w.append([])
		pose_orientations_z.append([])

		twist_linears_x.append([])
		twist_linears_y.append([])
		twist_angulars_z.append([])

		for j in range(len(tracks[i])):
			pose_positions_x[i].append(tracks[i][j].pose.pose.position.x)
			pose_positions_y[i].append(tracks[i][j].pose.pose.position.x)
			pose_orientations_z[i].append(tracks[i][j].pose.pose.orientation.z)
			pose_orientations_w[i].append(tracks[i][j].pose.pose.orientation.w)

			twist_linears_x[i].append(tracks[i][j].twist.twist.linear.x)
			twist_linears_y[i].append(tracks[i][j].twist.twist.linear.y)
			twist_angulars_z[i].append(tracks[i][j].twist.twist.angular.z)


def main():
	bag = rosbag.Bag(sys.argv[1])
	initialize_positions(bag)

	env = Environment(1, 3, 9, 9, Point(18.0, 10.0), Point(2.0, 10.0))
	env.initialize_environment()

	env.save_states('../data/states.npy')
	env.save_actions('../data/actions.npy')
	env.save_transitions('../data/transitions.npy')


main()

