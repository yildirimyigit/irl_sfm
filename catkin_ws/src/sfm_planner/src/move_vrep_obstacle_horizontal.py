#!/usr/bin/env python

import rospy

try:
    import vrep
except:
    print('BOOM')


vel = 0.85  # m/s
x_pose_start, x_pose_limit = -5.0, 5.0
y_pose = 6

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
 
if client_id != -1: # if we connected successfully
    print ('Connected to remote API server')
print('starting')

rospy.init_node('obstacle_mover', anonymous=True)
res, obstacle_handle = vrep.simxGetObjectHandle(client_id,'obstacle',vrep.simx_opmode_oneshot_wait)
x_pose = x_pose_start

hz = 20
rate = rospy.Rate(hz)
while not rospy.is_shutdown():
    returnCode = vrep.simxSetObjectPosition(client_id, obstacle_handle, -1, (x_pose, y_pose, 2), vrep.simx_opmode_oneshot)
    
    x_pose += vel/hz
    if abs(x_pose) > abs(x_pose_limit):
        x_pose = x_pose_start

    rate.sleep()

vrep.simxFinish(-1)
print('finito')
    







