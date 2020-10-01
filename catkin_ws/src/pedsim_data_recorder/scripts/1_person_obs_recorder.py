#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from pedsim_msgs.msg import TrackedPersons

from track import Track
import time
import numpy as np

ITH_TEST = 10
EAST = np.array([1.5, 2.5])
WEST = np.array([27.5, 2.5])
OBS = np.array([5.0+ITH_TEST, 2.0])


def make_vector(s, g):
    return g-s


def get_angle(a, b):
    angle = np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])
    return angle


def get_distance(p0, p1):
    return np.linalg.norm(p0 - p1)


class Recorder:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.callback)
        data_path_prefix = '/home/yigit/Documents/projects/irl_sfm/data/demonstrations/1_person_1_obs/' + str(ITH_TEST)+'/'
        data_path_suffix = str(int(time.time()))
        self.data_path = data_path_prefix + data_path_suffix
        self.demonstrations = {}
        self.current_track_ids = []
        self.current_tracks = {}

    def callback(self, msg):
        header = msg.header
        tracks = msg.tracks

        # removing old tracks ########
        track_ids_in_msg = []
        for track in tracks:
            track_ids_in_msg.append(track.track_id)

        for t in self.current_track_ids:
            if t not in track_ids_in_msg:
                self.current_track_ids.remove(t)
                self.current_tracks.pop(t)
        ##############################

        for track in tracks:
            tid = track.track_id
            if tid not in self.current_track_ids:
                self.current_track_ids.append(tid)
                t = Track()
                t.id = tid
                t.start_time = rospy.Time.now()
                v_x = track.twist.twist.linear.x
                t.goal, t.source = (WEST, EAST)
                t.dir_vector = make_vector(t.source, t.goal)
                self.current_tracks.update({tid: t})

            track_obj = self.current_tracks[tid]
            t_diff = int((rospy.Time.now() - track_obj.start_time).to_nsec() / 1000)  # milliseconds conversion
            cx, cy = track.pose.pose.position.x, track.pose.pose.position.y
            current_pose = np.array([cx, cy])
            d_g_x, d_g_y = track_obj.goal - current_pose
            d_o_x, d_o_y = OBS - current_pose
            
            vx, vy = track.twist.twist.linear.x, track.twist.twist.linear.y
            state = np.array([t_diff, d_g_x, d_g_y, d_o_x, d_o_y, cx, cy, vx, vy])

            print(str(tid) + ': ' + str(d_o_x) + ' ' + str(d_o_y))

            if tid in self.demonstrations.keys():
                self.demonstrations[tid].append(state)
            else:
                self.demonstrations.update({tid: [state]})

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


