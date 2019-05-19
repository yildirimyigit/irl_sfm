from bisect import bisect_left
import math
import random
import numpy as np
from utils import *


# This class initializes actions and states arrays and also has the transaction function
class Environment(object):
    # delta_distance : this represents the default distance that agents take
    # action_div : # of intervals to divide 180 degrees for actions
    # theta_human_div : # of intervals to divide 180 degrees for theta_human
    # theta_goal_div : # of intervals to divide 180 degrees for theta_goal
    def __init__(self, delta_distance, action_div, theta_human_div, theta_goal_div,
                 start_point, goal_point):
        self.delta_distance = delta_distance
        self.action_div = action_div
        self.theta_human_div = theta_human_div
        self.theta_goal_div = theta_goal_div
        self.start_point = start_point
        self.goal_point = goal_point

        self.th_arr = []
        self.dh_arr = []
        self.tg_arr = []
        self.dg_arr = []

        # This array will have states in each four dimensioned
        # states[th][dh][tg][dg] = s --> th, dh ..etc represents indexes of theta_human, distance_human.. etc
        self.states = [[[[]]]]
        self.actions = []

        self.initialize_actions()

    # actions array should start from -90 to +90 degrees thus if divided by 5:
    # | -72 | -36 | 0 | 36 | 72 | -> 1-(1/10) + i*1/5
    def initialize_actions(self):
        change = 1.0 / self.action_div  # the beginning should be in the middle
        for i in range(self.action_div):
            # it is multiplied with pi in order to give it in radians format
            self.actions.append(Action((-1 + change / 2.0 + i * change) * math.pi))

    def random_state(self):
        return State(random.choice(self.th_arr), random.choice(self.dh_arr),
                     random.choice(self.tg_arr), random.choice(self.dg_arr), )

    # This method returns a new state for a given action, according to this in
    # initialize_states() states array is initialized for each dimension
    def transaction(self, state, action):
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

        s = self.states[thn_index][dhn_index][tgn_index][dgn_index]
        print('New state belongs to states[%d][%d][%d][%d] which has State(th:%f, dh:%f, tg:%f, dg:%f)'
              % (thn_index, dhn_index, tgn_index, dgn_index, s.th, s.dh, s.tg, s.dg))

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

    def initialize_states(self):
        for th in range(len(self.th_arr)):
            for dh in range(len(self.dh_arr)):
                for tg in range(len(self.tg_arr)):
                    for dg in range(len(self.dg_arr)):
                        self.states[th][dh][tg].append(
                            State(self.th_arr[th], self.dh_arr[dh], self.tg_arr[tg], self.dg_arr[dg]))
                    self.states[th][dh].append([])
                self.states[th].append([[]])
            self.states.append([[[]]])

    def initialize_fields(self):
        human_change = 1.0 / self.theta_human_div
        for i in range(self.theta_human_div):
            self.th_arr.append((human_change / 2.0 + i * human_change) * math.pi)

        goal_change = 1.0 / self.theta_goal_div
        for i in range(self.theta_goal_div):
            self.tg_arr.append((goal_change / 2.0 + i * goal_change) * math.pi)

        goal_current_distance = self.calculate_max_distance()
        self.dg_arr.append(goal_current_distance)
        while goal_current_distance > 0 and (not goal_current_distance < 0.05):
            change = goal_current_distance / 10.0
            goal_current_distance = goal_current_distance - change
            self.dg_arr.insert(0, goal_current_distance)

        human_current_distance = self.calculate_max_distance()
        self.dh_arr.append(human_current_distance)
        while human_current_distance > 0 and (not human_current_distance < 0.05):
            change = human_current_distance / 10.0
            human_current_distance = human_current_distance - change
            self.dh_arr.insert(0, human_current_distance)

    def calculate_max_distance(self):
        return ((self.start_point.x - self.goal_point.x) ** 2 +
                (self.start_point.y - self.goal_point.y) ** 2) ** (1.0 / 2.0)

    def print_fields(self):
        print("th_arr: ", self.th_arr)
        print("dh_arr: ", self.dh_arr)
        print("tg_arr: ", self.tg_arr)
        print("dg_arr: ", self.dg_arr)

    def print_states(self):
        for th in range(len(self.states)):
            for dh in range(len(self.states[th])):
                for tg in range(len(self.states[th][dh])):
                    for dg in range(len(self.states[th][dh][tg])):
                        s = self.states[th][dh][tg][dg]

                        print('states[%d][%d][%d][%d] is: State(th:%f, dh:%f, tg:%f, dg:%f)' % (
                            th, dh, tg, dg, s.th, s.dh, s.tg, s.dg))

    # Creates a linear array with states enumerated
    # enumeration is like: 00001 - 00002 - 00003 .... 0010 - 0011 - 0011 -...
    def save_states(self, file_name):
        linear_states = []
        for th in range(len(self.states)):
            for dh in range(len(self.states[th])):
                for tg in range(len(self.states[th][dh])):
                    for dg in range(len(self.states[th][dh][tg])):
                        linear_states.append(self.states[th][dh][tg][dg])
        linear_states = np.array(linear_states)
        np.save(file_name, linear_states)

    def load_states(self, file_name):
        with open(file_name, 'r') as f:
            f.seek(0)
            states = np.load(f)

        return states
