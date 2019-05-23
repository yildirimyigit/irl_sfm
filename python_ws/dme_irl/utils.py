# This class is only for representing start and goal points
class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class State(object):
    def __init__(self, distance_goal=0, theta_goal=0, distance_human=0, theta_human=0):
        self.th = theta_human
        self.dh = distance_human
        self.tg = theta_goal
        self.dg = distance_goal

    def is_equal(self, state):
        return state.th == self.th and state.dh == self.dh and state.tg == self.tg and state.dg == self.dg


# mean degree is represented with radians
class Action(object):
    def __init__(self, middle_degree):
        self.middle_degree = middle_degree

    def get_degree(self):
        return self.middle_degree
