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

        self.state_list = []    # Will replace linear_states, th_arr, dg_arr... and states
        self.action_list = []
        self.th_arr = []
        self.dh_arr = []
        self.tg_arr = []
        self.dg_arr = []

        # This array will have states in each four dimensioned
        # states[th][dh][tg][dg] = s --> th, dh ..etc represents indexes of theta_human, distance_human.. etc
        self.states = [[[[]]]]
        self.actions = []
        self.linear_states = np.array([])

        # self.initialize_actions()

    # actions array should start from -90 to +90 degrees thus if divided by 5:
    # | -72 | -36 | 0 | 36 | 72 | -> 1-(1/10) + i*1/5
    def initialize_actions(self):
        change = 1.0 / self.action_div  # the beginning should be in the middle
        for i in range(self.action_div):
            # it is multiplied with pi in order to give it in radians format
            self.action_list.append(Action((-1 + change / 2.0 + i * change) * math.pi))

    def initialize_states2(self):
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

    # TODO: Change to reflect the new data structure
    def random_state(self):
        return State(random.choice(self.th_arr), random.choice(self.dh_arr),
                     random.choice(self.tg_arr), random.choice(self.dg_arr), )

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
            change = goal_current_distance / 2
            goal_current_distance = goal_current_distance - change
            self.dg_arr.insert(0, goal_current_distance)

        human_current_distance = self.calculate_max_distance()
        self.dh_arr.append(human_current_distance)
        while human_current_distance > 0 and (not human_current_distance < 0.05):
            change = human_current_distance / 2
            human_current_distance = human_current_distance - change
            self.dh_arr.insert(0, human_current_distance)

        print(str(len(self.dg_arr))+'\n'+str(len(self.tg_arr))+'\n'+str(len(self.dh_arr))+'\n'+str(len(self.th_arr)))

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
        print('+ Environment.save_states()')
        linear_states = []
        for th in range(len(self.states)):
            for dh in range(len(self.states[th])):
                for tg in range(len(self.states[th][dh])):
                    for dg in range(len(self.states[th][dh][tg])):
                        linear_states.append(self.states[th][dh][tg][dg])
        self.linear_states = np.array(linear_states)
        np.save(file_name, self.linear_states)

    # save actions next to the states
    def save_actions(self, file_name):
        print('+ Environment.save_actions()')
        np.save(file_name, np.asarray(self.actions))

    def save_transitions(self, file_name):
        print('+ Environment.save_transitions()')
        nof_states = len(self.linear_states)
        print(nof_states)
        # transition_mat = np.zeros([nof_states, len(self.actions), nof_states], dtype=float)  # T[s][a][s']
        # print(np.shape(transition_mat))
        #
        # for i in range(nof_states):
        #     for j in range(len(self.actions)):
        #         s_prime = self.take_step(i, j)
        #         transition_mat[i, j, s_prime] = 1   # Set T(s, a, s') to 1, leave other dest 0
        #
        # np.save(file_name, transition_mat)

    def load_states(self, file_name):
        with open(file_name, 'r') as f:
            f.seek(0)
            states = np.load(f)
        return states

    def initialize_environment(self):
        print('+ Environment.initialize_environment()')
        self.initialize_states()
        self.initialize_actions()

    def take_step(self, sid, aid):
        state = self.linear_states[sid]
        action = self.actions[aid]
        (thn, dhn, tgn, dgn) = self.transition(state, action)
        return self.get_state_index(thn, dhn, tgn, dgn)

    def get_state_index(self, thn, dhn, tgn, dgn):
        thn_index = self.closest_index(thn, self.th_arr)
        dhn_index = self.closest_index(dhn, self.dh_arr)
        tgn_index = self.closest_index(tgn, self.tg_arr)
        dgn_index = self.closest_index(dgn, self.dg_arr)

        dhlen = len(self.dh_arr)
        tglen = len(self.tg_arr)
        dglen = len(self.dg_arr)

        tgeff = dglen
        dheff = tgeff * tglen
        theff = dhlen * dheff

        state_index = theff*thn_index + dheff*dhn_index + tgeff*tgn_index + dgn_index
        return state_index
