"""
  @author: yigit.yildirim@boun.edu.tr

  [1]: Kitani 2012, Activity Forecasting
"""
import numpy as np
from env import MDP
from neural_network import MyNN, sigm, linear, tanh, gaussian

import sys


# class RLAgent:
#     def __init__(self):
#         self.max_episode_steps = 1000
#         self.learning_rate = 0.9
#         self.epsilon = 0.25
#         self.max_episode = 100000
#         self.episode = 0
#         self.episode_reward = 0
#         self.episode_steps = 0
#
#         self.env = MDP()
#         self.state_id = self.env.get_start_state()
#         self.episode_rewards = np.zeros((self.max_episode, 1))
#
#         nof_actions = self.env.actions.shape[0]
#         nof_states = self.env.states.shape[0]
#         self.q_table = np.zeros((nof_states, nof_actions))
#
#     def act(self, action):
#         self.state_id, reward, goal_reached = self.env.step(self.env.states[self.state_id], action)
#         self.episode_steps += 1
#         self.episode_reward += reward
#         if goal_reached or self.episode_steps >= self.max_episode_steps:
#             self.restart_episode()
#             self.episode += 1
#             return reward, True
#         return reward, False
#
#     def restart_episode(self):
#         self.episode_rewards[self.episode] = self.episode_reward
#         print('Episode: {0} - Reward: {1}'.format(self.episode, self.episode_reward))
#         self.episode_reward = 0
#         self.episode_steps = 0
#         self.state_id = self.env.get_start_state()
#
#     def initialize_q(self):
#         self.q_table = np.random.rand(self.q_table.shape) / 100.0
#
#     def q_learn(self):
#         self.initialize_q()
#
#         while self.episode < self.max_episode:
#             episode_end = False
#             while not episode_end:
#                 chosen_action_id = self.choose_action()
#                 p_sid = self.state_id
#                 c_reward, episode_end = self.act(chosen_action_id)
#                 self.q_table[p_sid][chosen_action_id] += self.learning_rate * (c_reward + self.env.gamma * np.max(
#                     self.q_table[self.state_id][:]) - self.q_table[p_sid][chosen_action_id])
#
#     def choose_action(self):
#         if np.random.rand() < self.get_epsilon():  # epsilon-greedy
#             chosen_action = np.random.choice(range(self.env.actions.shape[0]))
#         else:
#             chosen_action = np.argmax(self.q_table[self.state_id][:])
#         return chosen_action
#
#     def get_epsilon(self):  # decaying epsilon
#         return np.max([0.05, self.epsilon*(1-(self.episode/self.max_episode))])


class IRLAgent:
    def __init__(self):
        self.env = MDP()
        self.rew_nn = MyNN(nn_arch=(4, 8, 8, 1), acts=[sigm, gaussian, tanh])  # initializes with random weights
        self.state_rewards = np.empty(len(self.env.states))

        self.state_id = self.env.start_id

        self.vi_loop = 100
        self.v = np.empty((len(self.env.states), self.vi_loop), dtype=float)
        self.q = np.empty((len(self.env.states), len(self.env.actions)), dtype=float)
        self.esvc = np.empty(len(self.env.states), dtype=float)
        self.esvc_mat = np.empty((len(self.env.states), self.vi_loop), dtype=float)

        self.emp_fc = 0
        self.calculate_emp_fc()

    ###############################################
    # [1]
    def backward_pass(self):
        print("+ IRLAgent.backward_pass")
        self.v[:] = -sys.float_info.max
        reward_vect = np.vectorize(self.reward)
        self.state_rewards = reward_vect(self.env.states)

        for i in range(self.vi_loop-1):
            self.v[self.env.goal_id, i] = 0

            for s in range(len(self.env.states)):
                if s == self.env.goal_id:
                    continue
                state_reward = self.state_rewards[s]
                for a in range(len(self.env.actions)):
                    self.q[s][a] = state_reward + np.matmul(self.env.transition[s][a], self.v[:, i])

            # v  = softmax_a Q
            q = np.exp(self.q)
            max_q = np.max(q, axis=1)   # max of each row: max q value over actions for each state
            nonzero_ids = np.where(max_q != 0)
            self.v[nonzero_ids, i+1] = max_q[nonzero_ids] / np.sum(q[nonzero_ids], axis=1)

            print('\rBackward Pass: {}'.format((i+1)), end='')
        print("\n- IRLAgent.backward_pass")

    ###############################################

    ###############################################
    # [1]
    # def backward_pass_wo_transition_matrix(self):
    #     self.v[:] = -np.inf
    #     for i in range(self.vi_loop):
    #         self.v[self.env.goal_id] = 0

    ###############################################

    ###############################################
    # [1]
    def forward_pass(self):  # esvc: expected state visitation count
        print("+ IRLAgent.forward_pass")
        self.esvc_mat[:, 0] = 0
        self.esvc_mat[self.env.start_id, :] = 1
        for i in range(self.vi_loop-1):
            self.esvc_mat[self.env.goal_id][i] = 0
            for j in range(len(self.env.states)):
                sumesvc = 0
                for k in range(len(self.env.states)):
                    for l in range(len(self.env.actions)):  # indices: s:j, s':k, a:l
                        sumesvc += self.env.transition[k, l, j]*self.policy(k, l)*self.esvc_mat[k, i]
                self.esvc_mat[j, i+1] = sumesvc

            print('\rForward Pass: {}'.format((i+1)), end='')
        self.esvc = np.sum(self.esvc_mat, axis=1)
        print("- IRLAgent.forward_pass")

    ###############################################

    def calculate_emp_fc(self):
        trajectories = np.load(self.env.path + 'trajectories.npy', encoding='bytes')
        sum_traj_feats = []
        for trajectory in trajectories:
            sum_traj_feats.append(np.sum(trajectory, axis=0))

        sum_all_feats = np.sum(sum_traj_feats, axis=0)
        self.emp_fc = sum_all_feats/len(trajectories)

    def exp_fc(self):   # expected feature counts
        return np.matmul(self.esvc.T, self.env.states)

    def policy(self, sid, aid):
        return np.exp(self.q[sid][aid] - self.v[sid, -1])   # last column in the v matrix

    def reward(self, state):
        return self.rew_nn.forward(np.asarray([state.dg, state.tg, state.dh, state.th]))
