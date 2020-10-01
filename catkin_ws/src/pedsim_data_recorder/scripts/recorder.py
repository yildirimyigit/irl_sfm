#!/usr/bin/env python
import rospy
from pedsim_msgs.msg import TrackedPersons

import numpy as np
import time


class Recorder:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.callback)
        data_path_prefix = '/home/yigit/Documents/projects/irl_sfm/data/demonstrations/'
        data_path_suffix = str(int(time.time()))
        self.data_path = data_path_prefix + data_path_suffix
        self.demonstrations = {}  # {1: [[x0, y0], [x1, y1], ...], 2: [[x0, y0], [x1, y1], ...], ...}

    def callback(self, msg):
        tracks = msg.tracks
        for track in tracks:
            pose_x = track.pose.pose.position.x
            pose_y = track.pose.pose.position.y
            pose_arr = np.array([pose_x, pose_y])
            if track.track_id in self.demonstrations.keys():
                self.demonstrations[track.track_id].append(pose_arr)
            else:
                self.demonstrations.update({track.track_id: [pose_arr]})

    def save(self):
        dems = []
        for dem in self.demonstrations.values():
            dems.append(np.array(dem))
        np.save(self.data_path, np.array(dems))


if __name__ == '__main__':
    rospy.init_node('pedsim_data_recorder', anonymous=True)
    r = Recorder()
    rospy.spin()
    r.save()
