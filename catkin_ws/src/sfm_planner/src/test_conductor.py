#!/usr/bin/env python

from sfm import SFMController
from recorder import Recorder

import os
import errno
import sys
import time
import numpy as np

import rospy
import roslaunch

try:
    import vrep
except:
    print('BOOM')


class Conductor:
    def __init__(self, root_path, cid):
        self.root_path = root_path
        rospy.set_param('/recorder_root_path', self.root_path)

        try:
            os.mkdir(self.root_path)
            os.mkdir(self.root_path + 'novel/')
        except OSError as exc:
            if exc.errno == errno.EEXIST:
                print('Folder exists. Moving on with \n', root_path)
                pass
            else:
                raise exc
                sys.exit(1)

        x = np.hstack((np.arange(-3.0, 0.6, 0.1), np.arange(1.5, 3.1, 0.1)))
        nx = np.array([1])
        y = np.arange(2, 10.1, 0.5)
        ny = np.array([6])
        self.obs_poses = np.array([[x0, y0] for x0 in x for y0 in y])

        self.novel_obs_pose = np.array([[x0, y0] for x0 in nx for y0 in ny])

        self.client_id = cid

    def execute(self):
        rospy.loginfo('executing...')
        res, obstacle_handle = vrep.simxGetObjectHandle(self.client_id,'obstacle',vrep.simx_opmode_oneshot_wait)

        # rate = rospy.Rate(20)  # hz

        package = 'sfm_planner'
        sfm_executable, recorder_executable = 'sfm.py', 'recorder.py'
        sfm_node, recorder_node = roslaunch.core.Node(package, sfm_executable), roslaunch.core.Node(package, recorder_executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        for obs_pose in np.vstack((self.obs_poses, self.novel_obs_pose)):
        #for obs_pose in self.novel_obs_pose:
            vrep.simxStartSimulation(self.client_id,vrep.simx_opmode_blocking)
            _, old_obs_pose = vrep.simxGetObjectPosition(self.client_id, obstacle_handle, -1, vrep.simx_opmode_oneshot)
            returnCode = vrep.simxSetObjectPosition(self.client_id, obstacle_handle, -1, (obs_pose[0], obs_pose[1], old_obs_pose[2]), vrep.simx_opmode_oneshot)
            
            rospy.loginfo('starting')
            launch.launch(recorder_node)
            rospy.loginfo('recorder started')
            time.sleep(0.5)
            process = launch.launch(sfm_node)
            rospy.loginfo('controller started')

            while process.is_alive():
                continue

            vrep.simxStopSimulation(self.client_id,vrep.simx_opmode_blocking)
            time.sleep(0.5)


if __name__ == '__main__':
    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')
    print('starting')

    rospy.init_node('test_conductor', anonymous=True)

    c = Conductor('/home/yigit/Documents/projects/irl_sfm/catkin_ws/src/sfm_planner/data/', clientID)
    c.execute()

    vrep.simxFinish(-1)
    print('finito')
