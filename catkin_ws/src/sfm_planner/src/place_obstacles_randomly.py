#!/usr/bin/env python

import rospy
import numpy as np
import time

try:
    import vrep
except:
    print('BOOM')


nof_obstacles = 10

x_limits, y_limits = (-3.0, 3.01), (2.0, 10.01)
x_range, y_range = (x_limits[1] - x_limits[0]), (y_limits[1] - y_limits[0])

x = np.arange(x_limits[0], x_limits[1], 0.5)
y = np.arange(y_limits[0], y_limits[1], 0.5)

possible_obstacle_poses = np.array([[x0, y0] for x0 in x for y0 in y])
obstacle_pose_ids = np.sort(np.random.choice(possible_obstacle_poses.shape[0], nof_obstacles, replace=False))

obstacle_poses = np.zeros((nof_obstacles, 2))
for i, pose_id in enumerate(obstacle_pose_ids):
    obstacle_poses[i, :] = possible_obstacle_poses[obstacle_pose_ids[i]]


vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
if client_id != -1: # if we connected successfully
    print ('Connected to remote API server')
print('starting')

rospy.init_node('obstacle_placer', anonymous=True)

for i in range(nof_obstacles):
    name = 'obstacle'+str(i)
    res, obstacle_handle = vrep.simxGetObjectHandle(client_id, name, vrep.simx_opmode_oneshot_wait)
    returnCode = vrep.simxSetObjectPosition(client_id, obstacle_handle, -1, (obstacle_poses[i, 0], obstacle_poses[i, 1], 2), vrep.simx_opmode_oneshot)
    print(str(returnCode) + ': ' + name + '->' + str(obstacle_poses[i, 0]) + 'x' + str(obstacle_poses[i, 1]))


time.sleep(1)
vrep.simxFinish(-1)
print('finito')
    



