#!/usr/bin/env python

from sfm import SFMController
from recorder import Recorder

import os
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
        try:
            os.mkdir(self.root_path)
            os.mkdir(self.root_path + 'novel/')
        except:
            pass
        
        x = np.array([-4, -3, -2, -1, 0, 2, 3, 4])
        nx = np.array([1])
        y = np.hstack((np.array(range(-10, 0)), np.array(range(1, 11))))
        ny = np.array([0])
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
            vrep.simxStartSimulation(self.client_id,vrep.simx_opmode_blocking)
            _, old_obs_pose = vrep.simxGetObjectPosition(self.client_id, obstacle_handle, -1, vrep.simx_opmode_oneshot)
            returnCode = vrep.simxSetObjectPosition(self.client_id, obstacle_handle, -1, (obs_pose[0], obs_pose[1], old_obs_pose[2]), vrep.simx_opmode_oneshot)
            
            # recorder = Recorder(obs=obs_pose, in_data_path=self.root_path)
            # sfm_controller = SFMController()
            # goal_reached = False
            
            #while not goal_reached:
            #    goal_reached = sfm_controller.execute()
            #    recorder.add_step()
            #    rate.sleep()
            
            launch.launch(recorder_node)
            process = launch.launch(sfm_node)
            
            while process.is_alive():
                continue
            
            vrep.simxStopSimulation(self.client_id,vrep.simx_opmode_blocking)
            time.sleep(1)
        
        
if __name__ == '__main__':
    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
     
    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')
    print('starting')
    
    rospy.init_node('test_conductor', anonymous=True)
    
    c = Conductor('/home/yigit/Documents/projects/irl_sfm/data/demonstrations/sfm/1_obs_huge/', clientID)
    c.execute()
    
    vrep.simxFinish(-1)
    print('finito')