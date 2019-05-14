"""
  @author: yigit.yildirim@boun.edu.tr
"""

import numpy as np
from agent import IRLAgent


class DME:
    def __init__(self):
        self.irl_agent = IRLAgent()
        self.iter_count = 100

    def dme(self):
        for i in range(self.iter_count):
            # calculate state rewards
            reward_vect = np.vectorize(self.irl_agent.reward)
            self.irl_agent.state_rewards = reward_vect(self.irl_agent.env.states)

            # solve mdp wrt current reward
            self.irl_agent.backward_pass()
            self.irl_agent.forward_pass()

            # calculate loss
            loss = self.irl_agent.emp_fc - self.irl_agent.exp_fc()
            self.irl_agent.rew_nn.backprop_diff(loss)















