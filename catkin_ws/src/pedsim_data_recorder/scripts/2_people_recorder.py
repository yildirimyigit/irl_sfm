#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from pedsim_msgs.msg import TrackedPersons

from track import Track
import time
import numpy as np


EAST = np.array([1.5, 2.5])
WEST = np.array([27.5, 2.5])


def make_vector(s, g):
    return g-s


def get_angle(a, b):
    angle = np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])
#    if angle > np.pi:
#        angle = angle - np.pi
#    elif angle < -np.pi:
#        angle += 2 * np.pi
    return angle


def get_distance(p0, p1):
    return np.linalg.norm(p0 - p1)


class Recorder:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.callback)
        data_path_prefix = '/home/yigit/Documents/projects/irl_sfm/data/demonstrations/2_people_sources_sinks/'
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
                t.goal, t.source = (EAST, WEST) if v_x < 0 else (WEST, EAST)
                t.dir_vector = make_vector(t.source, t.goal)
                self.current_tracks.update({tid: t})

            track_obj = self.current_tracks[tid]
            t_diff = int((rospy.Time.now() - track_obj.start_time).to_nsec() / 1000)  # milliseconds conversion
            cx, cy = track.pose.pose.position.x, track.pose.pose.position.y
            current_pose = np.array([cx, cy])
            # d_g = get_distance(current_pose, track_obj.goal)
            # th_g = get_angle(make_vector(current_pose, track_obj.goal), track_obj.dir_vector)
            d_g_x, d_g_y = track_obj.goal - current_pose
            for other_track in tracks:
                if other_track.track_id != tid:  # if more than one other track, change this part
                    other_pose = np.array([other_track.pose.pose.position.x, other_track.pose.pose.position.y])
                    d_h = get_distance(current_pose, other_pose)
                    cur_v = make_vector(current_pose, other_pose)
                    th_h = np.abs(get_angle(cur_v, track_obj.dir_vector))
                    if th_h > np.pi/2:
                        d_h *= -1
                    #    th_h = np.pi - np.abs(th_h)
            vx, vy = track.twist.twist.linear.x, track.twist.twist.linear.y
            state = np.array([t_diff, d_g_x, d_g_y, d_h, np.sin(th_h), np.cos(th_h), cx, cy, vx, vy])

            if tid%2 == 0:
               #print('sin g: ' + str(np.sin(th_g)) + ' cos g: ' + str(np.cos(th_g)) + 'sin h: ' + str(np.sin(th_h)) + ' cos h: ' + str(np.cos(th_h)))
               print(d_g_x, d_g_y)

            if tid in self.demonstrations.keys():
                self.demonstrations[tid].append(state)
            else:
                self.demonstrations.update({tid: [state]})


#        if track.twist.twist.linear.x > 0:
#            goal_west = True
#        else:
#            goal_west = False
#             pose_x = track.pose.pose.position.x
#             pose_y = track.pose.pose.position.y
#             pose_arr = np.array([pose_x, pose_y])
#             if track.track_id in self.demonstrations.keys():
#                 self.demonstrations[track.track_id].append(pose_arr)
#             else:
#                 self.demonstrations.update({track.track_id: [pose_arr]})

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


