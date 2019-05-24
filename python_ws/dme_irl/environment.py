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
        print('+ Environment.initialize_states()')
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

    # This method returns the probability distribution on the state space which corresponds to the probabilities of
    # the agent's being on each state when it takes the given action in given state.
    def transition(self, state, action):
        print('+ Environment.transition')
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

        # print('Old state was: State(dg:%f, tg:%f, dh:%f, th:%f)' % (state.dg, state.tg, state.dh, state.th))
        # print('New state is: State(dg:%f, tg:%f, dh:%f, th:%f)' % (dgn, tgn, dhn, thn))

        state_prob_dist = np.zeros(len(self.state_list))

        # This part is specific to the deterministic environments
        state_prob_dist[self.find_closest_state(State(dgn, tgn, dhn, thn))] = 1
        return state_prob_dist

    def find_closest_state(self, state):
        print('+ Environment.find_closest_state')
        dg_ind = tg_ind = dh_ind = th_ind = -1
        dg_found = tg_found = dh_found = th_found = False
        for i in range(len(self.state_list)):
            if th_ind == -1 and state.th <= self.state_list[i].th:
                if i == 0:
                    th_ind = 0
                else:
                    th_ind = i if np.abs(state.th - self.state_list[i].th) < \
                                  np.abs(state.th - self.state_list[i-1].th) else (i-1)  # discretizing to the closest
                th_found = True
            if dh_ind == -1 and state.dh <= self.state_list[i].dh:
                if i == 0:
                    dh_ind = 0
                else:
                    dh_ind = i if np.abs(state.dh - self.state_list[i].dh) < \
                                  np.abs(state.dh - self.state_list[i-1].dh) else (i-1)
                    dh_found = True
            if tg_ind == -1 and state.tg <= self.state_list[i].tg:
                if i == 0:
                    tg_ind = 0
                else:
                    tg_ind = i if np.abs(state.tg - self.state_list[i].tg) < \
                                  np.abs(state.tg - self.state_list[i-1].tg) else (i-1)
                tg_found = True
            if dg_ind == -1 and state.dg <= self.state_list[i].dg:
                if i == 0:
                    dg_ind = 0
                else:
                    dg_ind = i if np.abs(state.dg - self.state_list[i].dg) < \
                                  np.abs(state.dg - self.state_list[i-1].dg) else (i-1)
                dg_found = True

            if dg_found and tg_found and dh_found and th_found:
                break

        # if not found, field is discretized into the last cell
        if not dg_found:
            dg_ind = len(self.state_list)-1
        if not tg_found:
            tg_ind = len(self.state_list)-1
        if not dh_found:
            dh_ind = len(self.state_list)-1
        if not th_found:
            th_ind = len(self.state_list)-1

        s = State(distance_goal=self.state_list[int(dg_ind)].dg, theta_goal=self.state_list[int(tg_ind)].tg,
                  distance_human=self.state_list[int(dh_ind)].dh, theta_human=self.state_list[int(th_ind)].th)

        print("Closest State:")
        print_state(s)

        for i in range(len(self.state_list)):
            if s.is_equal(self.state_list[i]):
                return i

        print('Error: ')
        print_state(s)
        raise ValueError('Environment.find_closest_state failed to find the matching state')

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

        s = a = 0
        print('****************state-action-transition***************')
        print_state(self.state_list[s])
        print_action(self.action_list[a])
        test = self.transition(self.state_list[s], self.action_list[a])
        for i in range(len(test)):
            if test[i] > 0:
                print(i, ": ", test[i])
        
        # for i in range(nof_states):
        #     for j in range(len(self.action_list)):
        #         transition_mat[i, j, :] = self.transition(self.state_list[i], self.action_list[j])
        #
        # np.save(file_name, transition_mat)

    def initialize_environment(self):
        print('+ Environment.initialize_environment()')
        self.initialize_states()
        self.initialize_actions()


def print_state(s):
        print('dg: {0}, tg: {1}, dh: {2}, th: {3}'.format(s.dg, s.tg, s.dh, s.th))


def print_action(a):
    print('mid_deg: {0}'.format(a.middle_degree))
