"""
  @author: yigit.yildirim@boun.edu.tr
  @author: irmak.guzey@boun.edu.tr
"""
import numpy as np
from utils import *
from environment import *


class MDP:
    def __init__(self, path='../../data/'):
        self.path = path
        self.states, self.actions, self.transition = self.create_env()
        # self.th_arr, self.dh_arr, self.tg_arr, self.dg_arr = self.get_discritized_env_arrays()
        self.delta_distance = 0.2
        self.gamma = 0.9
        # TODO: start and goal states
        self.start_id = 3074  # self.get_start_state()
        self.goal_id = 972

    # returns previously generated states and actions
    def create_env(self):
        return np.load(self.path + 'states.npy'), np.load(self.path + 'actions.npy'), \
               np.load(self.path + 'transitions.npy')

    # def get_discritized_env_arrays(self):
    #     return np.load(self.path + 'th.npy'), np.load(self.path + 'dh.npy'), \
    #            np.load(self.path + 'tg.npy'), np.load(self.path + 'dg.npy')

    # This method returns a new state for a given action, according to this in
    # initialize_states() states array is initialized for each dimension
    def step(self, sid, aid):
        new_state = self.take_step(sid, aid)
        rew = self.reward(sid)
        is_terminal = is_goal(sid)
        return new_state, rew, is_terminal

    def take_step(self, sid, aid):
        state = self.states[sid]
        action = self.actions[aid]
        dhx = state.dh * np.cos(state.th)
        dhy = state.dh * np.sin(state.th)
        dgx = state.dg * np.cos(state.tg)
        dgy = state.dg * np.sin(state.tg)

        # dgyn stands for new distance goal (y), the same for the rest of the variables as well
        dgxn = dgx - self.delta_distance / 2.0 * np.cos(action.middle_degree)
        dgyn = dgy - self.delta_distance / 2.0 * np.sin(action.middle_degree)
        tgn = np.arctan(dgyn / dgxn)
        dgn = (dgxn ** 2 + dgyn ** 2) ** (1.0 / 2.0)

        dhxn = dhx - self.delta_distance * np.cos(action.middle_degree)
        dhyn = dhy - self.delta_distance * np.sin(action.middle_degree)
        thn = np.arctan(dhyn / dhxn)
        if thn < 0:
            thn = np.pi + thn  # when the degree between people are negative
        dhn = (dhxn ** 2 + dhyn ** 2) ** (1.0 / 2.0)

        new_sid = self.get_state_index(thn, dhn, tgn, dgn)

        return self.states[new_sid]

    def get_state_index(self, thn, dhn, tgn, dgn):
        # thn_index = closest_index(thn, self.th_arr)
        # dhn_index = closest_index(dhn, self.dh_arr)
        # tgn_index = closest_index(tgn, self.tg_arr)
        # dgn_index = closest_index(dgn, self.dg_arr)
        #
        # dhlen = len(self.dh_arr)
        # tglen = len(self.tg_arr)
        # dglen = len(self.dg_arr)

        # tgeff = dglen
        # dheff = tgeff * tglen
        # theff = dhlen * dheff
        #
        # state_index = theff*thn_index + dheff*dhn_index + tgeff*tgn_index + dgn_index

        state_index = 0
        return state_index

    def get_start_state(self):
        return len(self.states)-1

    # R(s)
    def reward(self, state):
        if is_goal(state):
            rew = 100
        else:
            rew = -0.1  # The cost of a non-goal step
            if 0 < state.dh < 4:
                rew -= 10/state.dh**2
            elif state.dh == 0:
                rew = -100
        return rew


# Given state is goal or not
def is_goal(state):
    return state.dg == 0


# This returns the closest index for the check_element at the check_array
def closest_index(check_element, check_array):
    return (np.abs(check_array - check_element)).argmin()
