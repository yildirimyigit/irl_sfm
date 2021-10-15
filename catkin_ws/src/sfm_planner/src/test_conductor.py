#!/usr/bin/env python3

from sfm import SFMController
from recorder import Recorder

import os
import os.path
import errno
import sys
import time
import numpy as np

import rospy
import roslaunch

try:
    import sim
except:
    print('BOOM')


class Conductor:
    def __init__(self, cid, node_name):
        self.num_max_tests = int(1e6)

        self.source = [-3, 3]
        self.goal = [-3, 3]
        self.obs_x = [-3, 3]
        self.obs_y = [2, 10]

        self.client_id = cid
        self.scenario = rospy.get_param('/scenario')
        self.root_path = rospy.get_param('/recorder_root_path')

        self.node_name = node_name

        try:
            os.mkdir(self.root_path)
            os.mkdir(self.root_path + self.scenario + '/')
        except OSError as exc:
            if exc.errno == errno.EEXIST:
                print('Folder exists. Moving on with \n', self.root_path)
                pass
            else:
                raise exc
                sys.exit(1)

    def get_test_config(self):
        source_x = np.round(np.random.uniform(self.source[0], self.source[1]), 2)
        goal_x = np.round(np.random.uniform(self.goal[0], self.goal[1]), 2)
#        obs_x = np.round(np.random.uniform(self.obs_x[0], self.obs_x[1]), 2)
        obs_x = np.round(np.random.normal((source_x+goal_x)/2), 2)
        obs_y = np.round(np.random.uniform(self.obs_y[0], self.obs_y[1]), 2)

        filename = str(source_x) + '_' + str(goal_x) + '_' + str(obs_x) + '_' + str(obs_y) + '.npy'
        save_path = self.root_path + self.scenario + '/' + filename

        while os.path.isfile(save_path):  # if file exists, that config is used earlier
            source_x = np.round(np.random.uniform(self.source[0], self.source[1]), 2)
            goal_x = np.round(np.random.uniform(self.goal[0], self.goal[1]), 2)
#            obs_x = np.round(np.random.uniform(self.obs_x[0], self.obs_x[1]), 2)
            obs_x = np.round(np.random.normal((source_x+goal_x)/2), 2)
            obs_y = np.round(np.random.uniform(self.obs_y[0], self.obs_y[1]), 2)

            filename = str(source_x) + '_' + str(goal_x) + '_' + str(obs_x) + '_' + str(obs_y) + '.npy'
            save_path = self.root_path + self.scenario + '/' + filename

        return source_x, goal_x, obs_x, obs_y

    def execute(self):
        rospy.loginfo('executing...')
        res, obstacle_handle = sim.simxGetObjectHandle(self.client_id, 'obstacle', sim.simx_opmode_oneshot_wait)
        res, robotino_handle = sim.simxGetObjectHandle(self.client_id, 'robotino', sim.simx_opmode_oneshot_wait)
        res, goal_handle = sim.simxGetObjectHandle(self.client_id, 'GOAL', sim.simx_opmode_oneshot_wait)

        # rate = rospy.Rate(20)  # hz

        package = 'sfm_planner'
        sfm_executable, recorder_executable = 'sfm.py', 'recorder.py'
        sfm_node = roslaunch.core.Node(package, sfm_executable, namespace=self.node_name)
#        recorder_node = roslaunch.core.Node(package, recorder_executable, namespace=self.node_name, output="screen")
        recorder_node = roslaunch.core.Node(package, recorder_executable, namespace=self.node_name)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        for test_id in range(self.num_max_tests):
        #for test_id in range(3):
            source_x, goal_x, obs_x, obs_y = self.get_test_config()
            rospy.set_param('start/position/x', float(source_x))
            rospy.set_param('goal/position/x', float(goal_x))
            rospy.set_param('obstacle/position/x', float(obs_x))
            rospy.set_param('obstacle/position/y', float(obs_y))

            _, old_obs_pose = sim.simxGetObjectPosition(self.client_id, obstacle_handle, -1, sim.simx_opmode_oneshot)
            returnCode = sim.simxSetObjectPosition(self.client_id, obstacle_handle, -1, [obs_x, obs_y, 2.0], sim.simx_opmode_oneshot)
            _, old_robotino_pose = sim.simxGetObjectPosition(self.client_id, robotino_handle, -1, sim.simx_opmode_oneshot)
            returnCode = sim.simxSetObjectPosition(self.client_id, robotino_handle, -1, [source_x, 0, old_robotino_pose[2]], sim.simx_opmode_oneshot)
            _, old_goal_pose = sim.simxGetObjectPosition(self.client_id, goal_handle, -1, sim.simx_opmode_oneshot)
            returnCode = sim.simxSetObjectPosition(self.client_id, goal_handle, -1, [goal_x, 13, old_goal_pose[2]], sim.simx_opmode_oneshot)

            sim.simxStartSimulation(self.client_id, sim.simx_opmode_blocking)
            rospy.loginfo('test ' + str(test_id) + ' starting')
            rospy.loginfo('s: ' + str(source_x) + ' g: ' + str(goal_x) + ' obs: ' + str(obs_x) + '-' + str(obs_y))
            launch.launch(recorder_node)
            rospy.loginfo('recorder started')
            time.sleep(0.5)
            process = launch.launch(sfm_node)
            rospy.loginfo('controller started')

            while process.is_alive():
                continue

            sim.simxStopSimulation(self.client_id, sim.simx_opmode_blocking)
            time.sleep(0.5)


if __name__ == '__main__':
    # parsing node args
    args = rospy.myargv(argv=sys.argv)
    node_name = args[1]
    port = int(args[2])

    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 500, 5)

    if clientID != -1:  # if we connected successfully
        rospy.logwarn('Connected to remote API server')
        # res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        # rospy.logerr(objs)
        # time.sleep(10)

    rospy.init_node('test_conductor', anonymous=True)
    rospy.logwarn('starting tests')

    c = Conductor(clientID, node_name)
    c.execute()

    sim.simxFinish(-1)
    rospy.logwarn('finito')

