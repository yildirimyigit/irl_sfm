#!/usr/bin/env python3

import os
import os.path
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


class OnlineRunner:
    def __init__(self, cid):
        self.num_max_runs = 1e6
        self.source = [-2, 2]
        self.goal = [-2, 2]
        self.obs_x = [-3, 3]
        self.obs_y = [2, 10]

        self.client_id = cid
        self.runs_path = '/home/yigit/phd/yigit_phd_thesis/cnmp/output/sfm/small_env_changing_s_g/1607473236/runs/'

    def get_test_config(self):
        source_x = np.round(np.random.uniform(self.source[0], self.source[1]), 2)
        goal_x = np.round(np.random.uniform(self.goal[0], self.goal[1]), 2)
        obs_x = np.round(np.random.uniform(self.obs_x[0], self.obs_x[1]), 2)
        obs_y = np.round(np.random.uniform(self.obs_y[0], self.obs_y[1]), 2)

        filename = str(source_x) + '_' + str(goal_x) + '_' + str(obs_x) + '_' + str(obs_y) + '.npy'
        save_path = self.runs_path + filename

        while os.path.isfile(save_path):  # if file exists, that config is used earlier
            source_x = np.round(np.random.uniform(self.source[0], self.source[1]), 2)
            goal_x = np.round(np.random.uniform(self.goal[0], self.goal[1]), 2)
            obs_x = np.round(np.random.uniform(self.obs_x[0], self.obs_x[1]), 2)
            obs_y = np.round(np.random.uniform(self.obs_y[0], self.obs_y[1]), 2)

            filename = str(source_x) + '_' + str(goal_x) + '_' + str(obs_x) + '_' + str(obs_y) + '.npy'
            save_path = self.runs_path + filename

        return source_x, goal_x, obs_x, obs_y

    def execute(self):
        rospy.loginfo('executing...')
        res, obstacle_handle = vrep.simxGetObjectHandle(self.client_id,'obstacle',vrep.simx_opmode_oneshot_wait)
        res, robotino_handle = vrep.simxGetObjectHandle(self.client_id,'robotino',vrep.simx_opmode_oneshot_wait)
        res, goal_handle = vrep.simxGetObjectHandle(self.client_id,'GOAL',vrep.simx_opmode_oneshot_wait)

        package = 'sfm_planner'
        run_executable = 'auto_online_runner.py'
        run_node = roslaunch.core.Node(package, run_executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        #for test_id in range(self.num_max_runs):
        for test_id in range(3):
            source_x, goal_x, obs_x, obs_y = self.get_test_config()
            rospy.set_param('start/position/x', float(source_x))
            rospy.set_param('goal/position/x', float(goal_x))
            rospy.set_param('obstacle/position/x', float(obs_x))
            rospy.set_param('obstacle/position/y', float(obs_y))

            _, old_obs_pose = vrep.simxGetObjectPosition(self.client_id, obstacle_handle, -1, vrep.simx_opmode_oneshot)
            returnCode = vrep.simxSetObjectPosition(self.client_id, obstacle_handle, -1, (obs_x, obs_y, 2.0), vrep.simx_opmode_oneshot)
            _, old_robotino_pose = vrep.simxGetObjectPosition(self.client_id, robotino_handle, -1, vrep.simx_opmode_oneshot)
            returnCode = vrep.simxSetObjectPosition(self.client_id, robotino_handle, -1, (source_x, 0, old_robotino_pose[2]), vrep.simx_opmode_oneshot)
            _, old_goal_pose = vrep.simxGetObjectPosition(self.client_id, goal_handle, -1, vrep.simx_opmode_oneshot)
            returnCode = vrep.simxSetObjectPosition(self.client_id, goal_handle, -1, (goal_x, 13, old_goal_pose[2]), vrep.simx_opmode_oneshot)

            vrep.simxStartSimulation(self.client_id,vrep.simx_opmode_blocking)
            rospy.loginfo('test ' + str(test_id) + ' starting')
            rospy.loginfo('s: ' + str(source_x) + ' g: ' + str(goal_x) + ' obs: ' + str(obs_x) + '-' + str(obs_y))

            process = launch.launch(run_node)
            rospy.loginfo('runner started')

            while process.is_alive():
                continue

            vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_blocking)
            time.sleep(0.5)


if __name__ == '__main__':
    # parsing node args
    args = rospy.myargv(argv=sys.argv)

    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', port, True, True, 500, 5)

    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')

    rospy.init_node('online_runner', anonymous=True)
    rospy.logwarn('starting tests')

    onr = OnlineRunner(clientID)
    onr.execute()

    vrep.simxFinish(-1)
    rospy.logwarn('finito')


