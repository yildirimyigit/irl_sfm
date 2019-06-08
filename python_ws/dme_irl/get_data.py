import rosbag
import sys
import numpy as np
from environment import Environment
from environment import Point
from environment import State

# README::
# This program gets .bag file name and topic name as argument:
# USAGE: python get_data.py <bag_file>

goal1 = [18.0, 10.0]
goal2 = [2.0, 10.0]


#
# :param p0 first point, form: [x, y]
# :param p1 second point, form: [x, y]
#
def calculate_distance(p0, p1):
	return np.sqrt((p0[0]-p1[0]) ** 2 + (p0[1]-p1[1]) ** 2)


def initialize_positions(bag):
	print('+ initialize_positions()')

	cur_pos = prev_pos = 0
	raw_trajectories = []  # list of (goal, trajectory) tuples
	one_trajectory = []  # list of [first, second] poses
	first = True
	goal_set = False

	goal = goal1

	for _, msg, _ in bag.read_messages():
		pprev_pos = prev_pos
		prev_pos = cur_pos
		cur_pos = msg.tracks[0].pose.pose.position.x

		if pprev_pos < prev_pos < cur_pos or cur_pos < prev_pos < pprev_pos:  # no turning back in either direction
			if not goal_set:
				if pprev_pos < prev_pos < cur_pos:
					goal = goal1
				else:
					goal = goal2
				goal_set = True
			first_human_pose = [msg.tracks[0].pose.pose.position.x, msg.tracks[0].pose.pose.position.y]
			second_human_pose = [msg.tracks[1].pose.pose.position.x, msg.tracks[1].pose.pose.position.y]
			one_trajectory.append([first_human_pose, second_human_pose])
		else:
			if not first:
				raw_trajectories.append((one_trajectory, goal))
				one_trajectory = []
				goal_set = False
			else:
				first = False

	return raw_trajectories


def save_trajectories(raw_trajectories, path, env):
	print('+ save_trajectories()')
	trajectories = []
	i = 0
	for raw_trajectory, goal in raw_trajectories:
		trajectory = []
		for poses in raw_trajectory:
			s = compute_state(poses[0], poses[1], goal, env)
			trajectory.append(np.asarray([s.dg, s.tg, s.dh, s.th]))
			print(np.asarray([s.dg, s.tg, s.dh, s.th]))

		trajectories.append(np.asarray(trajectory))
		print(i)
		print('')
		i += 1
	np.save(path, np.asarray(trajectories))


def compute_state(first, second, goal, env):
	dh = calculate_distance(first, second)
	dg = calculate_distance(first, goal)

	dhx = second[0] - first[0]
	dhy = second[1] - first[1]
	
	dgx = goal[0] - first[0]
	dgy = goal[1] - first[1]
	
	tg = np.arctan2(dgy, dgx)
	th = np.arctan2(dhy, dhx)

	return env.state_list[env.find_closest_state(
		State(distance_goal=dg, theta_goal=tg, distance_human=dh, theta_human=th))]


def main():
	bag = rosbag.Bag(sys.argv[1])
	raw_trajectories = initialize_positions(bag)

	env = Environment(1, 3, 3, 36, Point(goal1[0], goal1[1]), Point(goal2[0], goal2[1]))
	env.initialize_environment()

	env.save_states('../../data/states.npy')
	env.save_actions('../../data/actions.npy')
	env.save_transitions('../../data/transitions.npy')

	save_trajectories(raw_trajectories, '../../data/trajectories.npy', env)


main()

