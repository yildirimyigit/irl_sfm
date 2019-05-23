from bisect import bisect_left
import math
import random
import numpy as np

from utils import *

min_goal_dist = 0.1
min_human_dist = 0.15


# This class initializes actions and states arrays and also has the transaction function
class Environment(object):
    # delta_distance : this represents the default distance that agents take
    # action_div : # of intervals to divide 180 degrees for actions
    # theta_human_div : # of intervals to divide 180 degrees for theta_human
    # theta_goal_div : # of intervals to divide 180 degrees for theta_goal
    def __init__(self, delta_distance, action_div, theta_human_div, theta_goal_div, start_point, goal_point):
        self.delta_distance = delta_distance
        self.action_div = action_div
        self.theta_human_div = theta_human_div
        self.theta_goal_div = theta_goal_div
        self.start_point = start_point
        self.goal_point = goal_point

        self.state_list = []    # list of State objects
        self.action_list = []   # list of Action objects

    # actions array should start from -90 to +90 degrees thus if divided by 5:
    # | -72 | -36 | 0 | 36 | 72 | -> 1-(1/10) + i*1/5
    def initialize_actions(self):
        print('+ Environment.initialize_actions()')
        change = 1.0 / self.action_div  # the beginning should be in the middle
        for i in range(self.action_div):
            # it is multiplied with pi in order to give it in radians format
            self.action_list.append(Action((-1 + change / 2.0 + i * change) * math.pi))

    def initialize_states(self):
        print('+ Environment.initialize_states2()')
        human_change = 1.0 / self.theta_human_div
        goal_change = 1.0 / self.theta_goal_div

        # discretizing the distances in logarithmic scale
        current_goal_distance = min_goal_dist
        max_human_dist = max_goal_distance = self.calculate_max_distance()

        while current_goal_distance < max_goal_distance:
            for i in range(self.theta_human_div):
                th_change = (human_change / 2.0 + i * human_change) * math.pi
                current_human_dist = min_human_dist
                while current_human_dist < max_human_dist:
                    for j in range(self.theta_goal_div):
                        tg_change = (goal_change / 2.0 + j * goal_change) * math.pi
                        self.state_list.append(State(current_goal_distance, tg_change, current_human_dist, th_change))
                    current_human_dist *= 2
            current_goal_distance *= 2
        for s in self.state_list:
            print_state(s)

    def random_state(self):
        return np.random.choice(self.state_list)

    # This method returns a new state for a given action, according to this in
    # initialize_states() states array is initialized for each dimension
    # TODO: Change to make it return the state
    def transition(self, state, action):
        dhx = state.dh * math.cos(state.th)
        dhy = state.dh * math.sin(state.th)
        dgx = state.dg * math.cos(state.tg)
        dgy = state.dg * math.sin(state.tg)

        # dgyn stands for new distance goal (y), the same for the rest of the variables as well
        dgxn = dgx - self.delta_distance / 2.0 * math.cos(action.middle_degree)
        dgyn = dgy - self.delta_distance / 2.0 * math.sin(action.middle_degree)
        tgn = math.atan(dgyn / dgxn)
        dgn = (dgxn ** 2 + dgyn ** 2) ** (1.0 / 2.0)

        dhxn = dhx - self.delta_distance * math.cos(action.middle_degree)
        dhyn = dhy - self.delta_distance * math.sin(action.middle_degree)
        thn = math.atan(dhyn / dhxn)
        if thn < 0:
            thn = math.pi + thn  # when the degree between people are negative
        dhn = (dhxn ** 2 + dhyn ** 2) ** (1.0 / 2.0)

        print('Old state was: State(th:%f, dh:%f, tg:%f, dg:%f)' % (state.th, state.dh, state.tg, state.dg))
        print('New state is: State(th:%f, dh:%f, tg:%f, dg:%f)' % (thn, dhn, tgn, dgn))

        thn_index = self.closest_index(thn, self.th_arr)
        dhn_index = self.closest_index(dhn, self.dh_arr)
        tgn_index = self.closest_index(tgn, self.tg_arr)
        dgn_index = self.closest_index(dgn, self.dg_arr)

        # s = self.states[thn_index][dhn_index][tgn_index][dgn_index]
        # print('New state belongs to states[%d][%d][%d][%d] which has State(th:%f, dh:%f, tg:%f, dg:%f)'
        #       % (thn_index, dhn_index, tgn_index, dgn_index, s.th, s.dh, s.tg, s.dg))

        return thn_index, dhn_index, tgn_index, dgn_index

    # This returns the closest index for the check_element at the check_array
    # bisect_left uses binary search
    def closest_index(self, check_element, check_array):
        pos = bisect_left(check_array, check_element)
        if pos == 0:
            return pos
        if pos == len(check_array):
            return pos - 1
        before = check_array[pos - 1]
        after = check_array[pos]
        if after - check_element < check_element - before:
            return pos
        else:
            return pos - 1

    def calculate_max_distance(self):
        return ((self.start_point.x - self.goal_point.x) ** 2 +
                (self.start_point.y - self.goal_point.y) ** 2) ** (1.0 / 2.0)

    # Creates a linear array with states enumerated
    # enumeration is like: 00001 - 00002 - 00003 .... 0010 - 0011 - 0011 -...
    def save_states(self, file_name):
        print('+ Environment.save_states()')
        np.save(file_name, np.asarray(self.state_list))

    # save actions next to the states
    def save_actions(self, file_name):
        print('+ Environment.save_actions()')
        np.save(file_name, np.asarray(self.action_list))

    def save_transitions(self, file_name):
        print('+ Environment.save_transitions()')
        nof_states = len(self.state_list)
        transition_mat = np.zeros([nof_states, len(self.action_list), nof_states], dtype=float)  # T[s][a][s']
        print(np.shape(transition_mat))

        for i in range(nof_states):
            for j in range(len(self.action_list)):
                s_prime = self.transition(self.state_list[i], self.action_list[j])
                transition_mat[i, j, s_prime] = 1   # Set T(s, a, s') to 1, leave other dest 0

        np.save(file_name, transition_mat)

    def initialize_environment(self):
        print('+ Environment.initialize_environment()')
        self.initialize_states()
        self.initialize_actions()


def print_state(s):
        print('dg: {0}, tg: {1}, dh: {2}, th: {3}'.format(s.dg, s.tg, s.dh, s.th))
