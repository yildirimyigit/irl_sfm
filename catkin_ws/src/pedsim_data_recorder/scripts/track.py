import rospy
import numpy as np


class Track:
    def __init__(self):
        self.id = -1
        self.start_time = rospy.Time.now()
        self.goal = np.zeros((2, 1))
        self.source = np.zeros((2, 1))
        self.dir_vector = np.zeros((2, 1))
